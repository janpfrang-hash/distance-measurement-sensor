"""
Serial Plotter & Logger für ESP32 Linearpotentiometer
=====================================================
Anforderungen: pip install pyserial matplotlib numpy scipy

Stabil für 24h+ Dauerbetrieb bei 200 Hz.
- Numpy-Ringpuffer fuer Plot-Anzeige (fester RAM ~32 MB)
- CSV-Logging direkt auf Disk (unbegrenzt, ~500 MB/24h)
- Min/Max-Downsampling fuer Plot (max 2000 Punkte)
- Inkrementeller Rainflow-Akkumulator (Histogramm-Bins, kein Cycle-List-Wachstum)
- Thread-sichere Datenuebergabe, minimale Lock-Contention
- Fehlertoleranz bei USB-Wacklern
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import csv
import os
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import numpy as np
from scipy import stats as sp_stats


# ==================================================================
# Rainflow-Akkumulator (inkrementell, fester Speicher)
# ==================================================================
class RainflowAccumulator:
    """
    Persistenter Rainflow-Zaehler mit festem Speicherverbrauch.
    
    Statt alle Zyklen-Tupel zu speichern (unbegrenztes Wachstum),
    werden die Amplituden direkt in ein Histogramm eingetragen.
    Der Stack hat maximal so viele Eintraege wie es verschachtelte
    Lastbereiche gibt (typisch < 100, worst case einige Tausend).
    
    Speicher: Histogramm 200 Bins * 8 Bytes + Stack < 10 KB = konstant.
    """
    MIN_AMPLITUDE = 0.1    # mm, kleinere Zyklen ignorieren
    HIST_BINS = 200        # Anzahl Histogramm-Bins
    HIST_MAX_AMP = 50.0    # mm, maximale erwartete Amplitude

    def __init__(self):
        self.lock = threading.Lock()
        self.stack = []           # Umkehrpunkte-Stack
        self.last_val = None
        self.last_dir = 0         # +1 steigend, -1 fallend

        # Histogramm: feste Bins statt wachsende Liste
        self.bin_edges = np.linspace(0, self.HIST_MAX_AMP, self.HIST_BINS + 1)
        self.bin_counts = np.zeros(self.HIST_BINS, dtype=np.float64)
        self.total_count = 0.0
        self.max_amplitude = 0.0

        # Dirty-Flag: nur neu zeichnen wenn sich etwas geaendert hat
        self._dirty = False

    def feed(self, value: float):
        """Neuen Messwert einspeisen."""
        with self.lock:
            if self.last_val is None:
                self.last_val = value
                return

            diff = value - self.last_val
            if abs(diff) < 1e-9:
                return

            direction = 1 if diff > 0 else -1

            if self.last_dir == 0:
                self.last_dir = direction
                self.stack.append(self.last_val)
                self.last_val = value
                return

            if direction != self.last_dir:
                self.stack.append(self.last_val)
                self._extract_cycles()
                self.last_dir = direction

            self.last_val = value

    def _extract_cycles(self):
        """4-Punkt-Methode auf dem Stack."""
        while len(self.stack) >= 4:
            s0 = self.stack[-4]
            s1 = self.stack[-3]
            s2 = self.stack[-2]
            s3 = self.stack[-1]

            r_inner = abs(s1 - s2)
            r_outer_left = abs(s0 - s1)
            r_outer_right = abs(s2 - s3)

            if r_inner <= r_outer_left and r_inner <= r_outer_right:
                amp = r_inner / 2.0
                del self.stack[-3]
                del self.stack[-2]

                if amp >= self.MIN_AMPLITUDE:
                    # Direkt ins Histogramm eintragen
                    bin_idx = int(amp / self.HIST_MAX_AMP * self.HIST_BINS)
                    bin_idx = min(bin_idx, self.HIST_BINS - 1)
                    self.bin_counts[bin_idx] += 1.0
                    self.total_count += 1.0
                    if amp > self.max_amplitude:
                        self.max_amplitude = amp
                    self._dirty = True
            else:
                break

    def get_summary(self):
        """Thread-sicher: (total_count, max_amplitude) - leichtgewichtig."""
        with self.lock:
            return (self.total_count, self.max_amplitude)

    def get_histogram(self):
        """Thread-sicher: (bin_edges, bin_counts, dirty_flag).
        Gibt Kopien zurueck und setzt dirty auf False."""
        with self.lock:
            dirty = self._dirty
            self._dirty = False
            return (self.bin_edges.copy(), self.bin_counts.copy(), dirty)

    def clear(self):
        with self.lock:
            self.stack.clear()
            self.bin_counts[:] = 0
            self.last_val = None
            self.last_dir = 0
            self.total_count = 0.0
            self.max_amplitude = 0.0
            self._dirty = True


# ==================================================================
# Ringpuffer (numpy-basiert, fester RAM)
# ==================================================================
class RingBuffer:
    """
    Fester Ringpuffer fuer (time, value) Paare.
    2M Punkte = ~32 MB RAM = ~2.8 h bei 200 Hz im Plot-Puffer.
    """
    CAPACITY = 2_000_000

    def __init__(self):
        self.times = np.zeros(self.CAPACITY, dtype=np.float64)
        self.values = np.zeros(self.CAPACITY, dtype=np.float32)
        self.head = 0
        self.count = 0
        self.lock = threading.Lock()

    def append(self, t: float, v: float):
        with self.lock:
            self.times[self.head] = t
            self.values[self.head] = v
            self.head = (self.head + 1) % self.CAPACITY
            if self.count < self.CAPACITY:
                self.count += 1

    def get_window(self, t_min: float, t_max: float, max_points: int = 2000):
        """
        (times, values) im Bereich [t_min, t_max],
        downsampled auf max_points via Min/Max pro Bucket.
        """
        with self.lock:
            if self.count == 0:
                return np.array([]), np.array([])

            if self.count < self.CAPACITY:
                # Kein Wrap -> einfacher Slice
                t = self.times[:self.count]
                v = self.values[:self.count]
                mask = (t >= t_min) & (t <= t_max)
                t_win = t[mask].copy()
                v_win = v[mask].copy()
            else:
                # Wrap: statt teures concatenate, zwei Slices getrennt filtern
                # Teil 1: head..end, Teil 2: 0..head
                t1 = self.times[self.head:]
                v1 = self.values[self.head:]
                m1 = (t1 >= t_min) & (t1 <= t_max)

                t2 = self.times[:self.head]
                v2 = self.values[:self.head]
                m2 = (t2 >= t_min) & (t2 <= t_max)

                t_win = np.concatenate((t1[m1], t2[m2]))
                v_win = np.concatenate((v1[m1], v2[m2]))

        n = len(t_win)
        if n == 0:
            return np.array([]), np.array([])

        if n > max_points:
            step = n / max_points
            indices = []
            for i in range(max_points):
                s = int(i * step)
                e = int((i + 1) * step)
                chunk = v_win[s:e]
                if len(chunk) > 0:
                    idx_min = s + np.argmin(chunk)
                    idx_max = s + np.argmax(chunk)
                    if idx_min <= idx_max:
                        indices.extend([idx_min, idx_max])
                    else:
                        indices.extend([idx_max, idx_min])
            indices = sorted(set(indices))
            t_win = t_win[indices]
            v_win = v_win[indices]

        return t_win, v_win

    def get_latest(self):
        with self.lock:
            if self.count == 0:
                return None
            idx = (self.head - 1) % self.CAPACITY
            return float(self.times[idx]), float(self.values[idx])

    def clear(self):
        with self.lock:
            self.head = 0
            self.count = 0


# ==================================================================
# Hauptanwendung
# ==================================================================
class SerialPlotterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Poti Plotter & Logger")
        self.root.geometry("1100x900")
        self.root.minsize(800, 650)

        # State
        self.ser = None
        self.running = False
        self.read_thread = None
        self.start_time = None

        # Logging
        self.logging = False
        self.log_file = None
        self.csv_writer = None
        self.log_lock = threading.Lock()
        self.total_logged = 0
        self.log_flush_counter = 0
        self.log_filepath = None

        # Daten
        self.ring = RingBuffer()
        self.rf_acc = RainflowAccumulator()

        # Rainflow-Snapshot fuer CSV (wird nur bei Aenderung aktualisiert)
        self.rf_snap_count = 0.0
        self.rf_snap_max = 0.0

        # Statistik
        self.samples_received = 0
        self.errors_count = 0
        self.last_rate_time = 0.0
        self.last_rate_count = 0
        self.current_rate = 0.0

        self._build_ui()
        self._setup_plot()
        self._animate()

    # ================================================================
    # UI
    # ================================================================
    def _build_ui(self):
        conn_frame = ttk.LabelFrame(self.root, text="Verbindung", padding=6)
        conn_frame.pack(fill="x", padx=8, pady=(8, 2))

        ttk.Label(conn_frame, text="Port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            conn_frame, textvariable=self.port_var, width=25, state="readonly"
        )
        self.port_combo.pack(side="left", padx=4)

        self.scan_btn = ttk.Button(conn_frame, text="Ports scannen", command=self._scan_ports)
        self.scan_btn.pack(side="left", padx=4)

        ttk.Label(conn_frame, text="Baud:").pack(side="left", padx=(12, 0))
        self.baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(
            conn_frame, textvariable=self.baud_var, width=8, state="readonly",
            values=["9600", "19200", "38400", "57600", "115200", "230400"]
        )
        baud_combo.pack(side="left", padx=4)

        self.connect_btn = ttk.Button(conn_frame, text="Verbinden", command=self._toggle_connection)
        self.connect_btn.pack(side="left", padx=8)

        self.status_var = tk.StringVar(value="Getrennt")
        ttk.Label(conn_frame, textvariable=self.status_var, foreground="gray").pack(side="left", padx=8)

        # Anzeige
        ctrl_frame = ttk.LabelFrame(self.root, text="Anzeige", padding=6)
        ctrl_frame.pack(fill="x", padx=8, pady=2)

        ttk.Label(ctrl_frame, text="Zeitachse:").pack(side="left")
        self.time_unit_var = tk.StringVar(value="s")
        for unit in ("s", "min", "h"):
            ttk.Radiobutton(ctrl_frame, text=unit, variable=self.time_unit_var, value=unit).pack(side="left", padx=2)

        ttk.Label(ctrl_frame, text="    Zeitfenster:").pack(side="left", padx=(12, 0))
        self.window_var = tk.StringVar(value="30")
        ttk.Entry(ctrl_frame, textvariable=self.window_var, width=6).pack(side="left", padx=4)
        ttk.Label(ctrl_frame, text="(in gew. Einheit, 0=alles)").pack(side="left")

        ttk.Separator(ctrl_frame, orient="vertical").pack(side="left", fill="y", padx=12)

        ttk.Button(ctrl_frame, text="Plot löschen", command=self._clear_data).pack(side="left", padx=4)

        # Logging
        log_frame = ttk.LabelFrame(self.root, text="Logging", padding=6)
        log_frame.pack(fill="x", padx=8, pady=2)

        self.log_btn = ttk.Button(log_frame, text="Logging starten", command=self._toggle_logging)
        self.log_btn.pack(side="left", padx=4)

        ttk.Separator(log_frame, orient="vertical").pack(side="left", fill="y", padx=8)

        ttk.Button(log_frame, text="Rainflow Analyse Logfile",
                   command=self._analyze_logfile).pack(side="left", padx=4)

        self.log_status_var = tk.StringVar(value="Kein Log aktiv")
        ttk.Label(log_frame, textvariable=self.log_status_var, foreground="gray").pack(side="left", padx=8)

        # Statuszeile
        status_frame = ttk.Frame(self.root, padding=4)
        status_frame.pack(fill="x", padx=8, pady=0)

        self.info_var = tk.StringVar(value="")
        ttk.Label(status_frame, textvariable=self.info_var, foreground="gray").pack(side="left")

        self.rate_var = tk.StringVar(value="")
        ttk.Label(status_frame, textvariable=self.rate_var, foreground="gray").pack(side="right")

        # Plot
        self.plot_frame = ttk.Frame(self.root)
        self.plot_frame.pack(fill="both", expand=True, padx=8, pady=(2, 8))

    # ================================================================
    # Plot
    # ================================================================
    def _setup_plot(self):
        self.fig = Figure(figsize=(10, 7), dpi=100)
        self.fig.set_facecolor("#1e1e2e")

        gs = self.fig.add_gridspec(2, 2, height_ratios=[2, 1], hspace=0.35, wspace=0.3,
                                   left=0.08, right=0.97, top=0.97, bottom=0.07)

        # Zeitserie (oben, volle Breite)
        self.ax = self.fig.add_subplot(gs[0, :])
        self._style_ax(self.ax)
        self.ax.set_xlabel("Zeit [s]", color="#cdd6f4")
        self.ax.set_ylabel("Weg [mm]", color="#cdd6f4")
        (self.line,) = self.ax.plot([], [], color="#89b4fa", linewidth=1.0)

        # Q-Q Plot (unten links)
        self.ax_qq = self.fig.add_subplot(gs[1, 0])
        self._style_ax(self.ax_qq)
        self.ax_qq.set_xlabel("Theoret. Quantile", color="#cdd6f4", fontsize=9)
        self.ax_qq.set_ylabel("Messwert Quantile", color="#cdd6f4", fontsize=9)
        self.ax_qq.set_title("Q-Q Plot (Normalvert.)", color="#cdd6f4", fontsize=10)
        (self.qq_dots,) = self.ax_qq.plot([], [], "o", color="#f38ba8",
                                          markersize=2, alpha=0.7)
        (self.qq_line,) = self.ax_qq.plot([], [], "-", color="#a6e3a1",
                                          linewidth=1.0, alpha=0.8)

        # Rainflow-Histogramm (unten rechts)
        self.ax_rf = self.fig.add_subplot(gs[1, 1])
        self._style_ax(self.ax_rf)
        self.ax_rf.set_xlabel("Amplitude [mm]", color="#cdd6f4", fontsize=9)
        self.ax_rf.set_ylabel("Zyklen", color="#cdd6f4", fontsize=9)
        self.ax_rf.set_title("Rainflow: 0 Zyklen, max 0.000 mm",
                             color="#cdd6f4", fontsize=10)
        # Leere Bars initialisieren - werden nur bei dirty neugezeichnet
        self.rf_bar_container = None

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        NavigationToolbar2Tk(self.canvas, self.plot_frame).update()

    def _style_ax(self, ax):
        ax.set_facecolor("#1e1e2e")
        ax.tick_params(colors="#cdd6f4", labelsize=8)
        for spine in ax.spines.values():
            spine.set_color("#45475a")
        ax.grid(True, color="#45475a", alpha=0.5, linewidth=0.5)

    # ================================================================
    # Ports
    # ================================================================
    def _scan_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [f"{p.device} - {p.description}" for p in ports]
        self.port_combo["values"] = port_list
        if port_list:
            self.port_combo.current(0)
        else:
            self.port_var.set("")
            messagebox.showinfo("Ports", "Keine seriellen Schnittstellen gefunden.")

    # ================================================================
    # Verbindung
    # ================================================================
    def _toggle_connection(self):
        if self.running:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port_str = self.port_var.get()
        if not port_str:
            messagebox.showwarning("Port", "Bitte zuerst einen Port auswählen.")
            return
        port = port_str.split(" - ")[0].strip()
        baud = int(self.baud_var.get())

        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
            time.sleep(0.5)
            self.ser.reset_input_buffer()
        except serial.SerialException as e:
            messagebox.showerror("Fehler", f"Kann Port nicht öffnen:\n{e}")
            return

        self.running = True
        self.start_time = time.time()
        self.samples_received = 0
        self.errors_count = 0
        self.last_rate_time = time.time()
        self.last_rate_count = 0
        self.connect_btn.config(text="Trennen")
        self.status_var.set(f"Verbunden: {port} @ {baud}")

        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def _disconnect(self):
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=2)
            self.read_thread = None
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connect_btn.config(text="Verbinden")
        self.status_var.set("Getrennt")

    # ================================================================
    # Daten lesen (Thread)
    # ================================================================
    def _read_loop(self):
        buf = b""
        consecutive_errors = 0
        # Lokaler Snapshot der RF-Werte (wird nur bei feed aktualisiert)
        rf_count_local = 0.0
        rf_max_local = 0.0

        while self.running:
            try:
                if self.ser is None or not self.ser.is_open:
                    break

                waiting = self.ser.in_waiting
                if waiting > 0:
                    chunk = self.ser.read(min(waiting, 4096))
                    buf += chunk
                    consecutive_errors = 0

                    if len(buf) > 65536:
                        last_nl = buf.rfind(b"\n", 0, -1)
                        if last_nl > 0:
                            buf = buf[last_nl + 1:]

                    while b"\n" in buf:
                        line_bytes, buf = buf.split(b"\n", 1)
                        line = line_bytes.decode("utf-8", errors="ignore").strip()
                        if not line:
                            continue
                        try:
                            value = float(line)
                        except ValueError:
                            continue

                        t = time.time() - self.start_time
                        self.samples_received += 1

                        self.ring.append(t, value)
                        self.rf_acc.feed(value)

                        # RF-Snapshot fuer CSV nur bei Aenderung aktualisieren
                        # (get_summary ist leichtgewichtig: nur 2 Zahlen lesen)
                        new_count = self.rf_acc.total_count
                        if new_count != rf_count_local:
                            rf_count_local = self.rf_acc.total_count
                            rf_max_local = self.rf_acc.max_amplitude

                        if self.logging:
                            with self.log_lock:
                                if self.csv_writer:
                                    try:
                                        self.csv_writer.writerow(
                                            [f"{t:.4f}", f"{value:.4f}",
                                             f"{int(rf_count_local)}",
                                             f"{rf_max_local:.4f}"]
                                        )
                                        self.total_logged += 1
                                        self.log_flush_counter += 1
                                        if self.log_flush_counter >= 1000:
                                            self.log_file.flush()
                                            self.log_flush_counter = 0
                                    except Exception:
                                        self.errors_count += 1
                else:
                    time.sleep(0.002)

            except (serial.SerialException, OSError):
                consecutive_errors += 1
                self.errors_count += 1
                time.sleep(0.5)
                if consecutive_errors > 60:
                    self.running = False
                    break

    # ================================================================
    # Plot-Animation (10 FPS)
    # ================================================================
    def _animate(self):
        try:
            self._update_plot()
        except Exception:
            pass
        self.root.after(100, self._animate)

    def _update_plot(self):
        latest = self.ring.get_latest()
        if latest is None:
            return

        t_now = latest[0]
        unit = self.time_unit_var.get()
        divisor = {"s": 1.0, "min": 60.0, "h": 3600.0}[unit]

        try:
            window = float(self.window_var.get())
        except ValueError:
            window = 30

        if window > 0:
            window_s = window * divisor
            t_min = max(0, t_now - window_s)
            t_max = t_now
        else:
            t_min = 0
            t_max = t_now

        t_arr, v_arr = self.ring.get_window(t_min, t_max, max_points=2000)

        if len(t_arr) == 0:
            return

        t_display = t_arr / divisor

        # --- Zeitserie ---
        self.line.set_data(t_display, v_arr)
        x_min = t_min / divisor
        x_max = t_max / divisor
        if x_max <= x_min:
            x_max = x_min + 0.1
        self.ax.set_xlim(x_min, x_max)

        vmin = float(np.min(v_arr))
        vmax = float(np.max(v_arr))
        margin = max(0.05, (vmax - vmin) * 0.1)
        self.ax.set_ylim(vmin - margin, vmax + margin)
        self.ax.set_xlabel(f"Zeit [{unit}]", color="#cdd6f4")

        # --- Q-Q Plot ---
        if len(v_arr) >= 20:
            qq_data = v_arr
            if len(qq_data) > 500:
                idx = np.linspace(0, len(qq_data) - 1, 500, dtype=int)
                qq_data = qq_data[idx]

            qq_sorted = np.sort(qq_data)
            n = len(qq_sorted)
            theoretical = sp_stats.norm.ppf(
                (np.arange(1, n + 1) - 0.5) / n,
                loc=np.mean(qq_sorted),
                scale=max(np.std(qq_sorted), 1e-9)
            )

            self.qq_dots.set_data(theoretical, qq_sorted)

            q_min = min(theoretical[0], qq_sorted[0])
            q_max = max(theoretical[-1], qq_sorted[-1])
            self.qq_line.set_data([q_min, q_max], [q_min, q_max])

            self.ax_qq.set_xlim(theoretical[0] * 1.05, theoretical[-1] * 1.05)
            self.ax_qq.set_ylim(qq_sorted[0] - margin, qq_sorted[-1] + margin)
        else:
            self.qq_dots.set_data([], [])
            self.qq_line.set_data([], [])

        # --- Rainflow-Histogramm (nur neuzeichnen wenn dirty) ---
        bin_edges, bin_counts, dirty = self.rf_acc.get_histogram()
        total_count, max_amp = self.rf_acc.get_summary()
        num_cycles = int(total_count)

        if dirty or self.rf_bar_container is None:
            self.ax_rf.cla()
            self._style_ax(self.ax_rf)
            self.ax_rf.set_xlabel("Amplitude [mm]", color="#cdd6f4", fontsize=9)
            self.ax_rf.set_ylabel("Zyklen", color="#cdd6f4", fontsize=9)

            # Nur Bins mit Daten darstellen
            nonzero = bin_counts > 0
            if np.any(nonzero):
                # Sichtbaren Bereich bestimmen
                nz_indices = np.where(nonzero)[0]
                first_bin = max(0, nz_indices[0])
                last_bin = min(len(bin_counts), nz_indices[-1] + 2)
                visible_edges = bin_edges[first_bin:last_bin + 1]
                visible_counts = bin_counts[first_bin:last_bin]

                self.ax_rf.bar(
                    visible_edges[:-1], visible_counts,
                    width=np.diff(visible_edges),
                    align="edge",
                    color="#fab387", alpha=0.85, edgecolor="#45475a",
                    linewidth=0.5
                )

        self.ax_rf.set_title(
            f"Rainflow: {num_cycles} Zyklen, max {max_amp:.3f} mm",
            color="#cdd6f4", fontsize=10
        )

        self.canvas.draw_idle()

        # --- Statuszeile ---
        now = time.time()
        dt_rate = now - self.last_rate_time
        if dt_rate >= 1.0:
            self.current_rate = (self.samples_received - self.last_rate_count) / dt_rate
            self.last_rate_count = self.samples_received
            self.last_rate_time = now

        current_val = latest[1]
        elapsed = latest[0]
        h = int(elapsed // 3600)
        m = int((elapsed % 3600) // 60)
        s = int(elapsed % 60)

        info = (
            f"Samples: {self.samples_received:,}  |  "
            f"Aktuell: {current_val:.4f} mm  |  "
            f"Laufzeit: {h:02d}:{m:02d}:{s:02d}"
            f"  |  RF-Zyklen: {num_cycles}, max: {max_amp:.3f} mm"
        )
        if self.logging:
            log_size = self._get_log_size_str()
            info += f"  |  Geloggt: {self.total_logged:,} ({log_size})"
        if self.errors_count > 0:
            info += f"  |  Fehler: {self.errors_count}"
        self.info_var.set(info)
        self.rate_var.set(f"{self.current_rate:.0f} Hz")

    # ================================================================
    # Logging
    # ================================================================
    def _toggle_logging(self):
        if self.logging:
            self._stop_logging()
        else:
            self._start_logging()

    def _start_logging(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"poti_log_{timestamp}.csv"
        filepath = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("Alle", "*.*")],
            initialfile=default_name,
        )
        if not filepath:
            return

        try:
            with self.log_lock:
                self.log_file = open(filepath, "w", newline="", buffering=8192)
                self.csv_writer = csv.writer(self.log_file, delimiter=";")
                self.csv_writer.writerow(["Zeit_s", "Weg_mm", "RF_Zyklen", "RF_MaxAmplitude_mm"])
                self.total_logged = 0
                self.log_flush_counter = 0
                self.log_filepath = filepath
                self.logging = True
            self.log_btn.config(text="Logging stoppen")
            self.log_status_var.set(f"Log: {os.path.basename(filepath)}")
        except IOError as e:
            messagebox.showerror("Fehler", f"Kann Log-Datei nicht erstellen:\n{e}")

    def _stop_logging(self):
        with self.log_lock:
            self.logging = False
            if self.log_file:
                try:
                    self.log_file.flush()
                    self.log_file.close()
                except Exception:
                    pass
                self.log_file = None
            self.csv_writer = None
        size_str = self._get_log_size_str()
        self.log_btn.config(text="Logging starten")
        self.log_status_var.set(
            f"Log gespeichert ({self.total_logged:,} Samples, {size_str})"
        )
        self.log_filepath = None

    def _get_log_size_str(self):
        """Aktuelle Logfile-Groesse als lesbarer String."""
        if self.log_filepath and os.path.exists(self.log_filepath):
            try:
                size_bytes = os.path.getsize(self.log_filepath)
                if size_bytes < 1024:
                    return f"{size_bytes} B"
                elif size_bytes < 1024 * 1024:
                    return f"{size_bytes / 1024:.1f} KB"
                elif size_bytes < 1024 * 1024 * 1024:
                    return f"{size_bytes / (1024 * 1024):.1f} MB"
                else:
                    return f"{size_bytes / (1024 * 1024 * 1024):.2f} GB"
            except OSError:
                return "?"
        return "0 B"

    # ================================================================
    # Rainflow Analyse aus Logfile
    # ================================================================
    def _analyze_logfile(self):
        filepath = filedialog.askopenfilename(
            title="Poti-Logfile für Rainflow-Analyse auswählen",
            filetypes=[("CSV", "*.csv"), ("Alle", "*.*")]
        )
        if not filepath:
            return

        try:
            values = self._load_log_values(filepath)
        except Exception as e:
            messagebox.showerror("Fehler", f"Kann Datei nicht lesen:\n{e}")
            return

        if len(values) < 10:
            messagebox.showinfo("Analyse", "Zu wenige Datenpunkte in der Datei.")
            return

        # Rainflow offline berechnen
        cycles = self._rainflow_offline(values, min_amplitude=0.1)

        if not cycles:
            messagebox.showinfo("Analyse",
                                "Keine Zyklen mit Amplitude >= 0.1 mm gefunden.")
            return

        amps = np.array([c[0] for c in cycles])
        counts = np.array([c[2] for c in cycles])
        total = np.sum(counts)
        max_amp = np.max(amps)

        # Neues Fenster mit Ergebnis
        self._show_analysis_window(filepath, values, amps, counts,
                                   total, max_amp)

    @staticmethod
    def _load_log_values(filepath):
        """CSV einlesen, Weg-Spalte als numpy-Array zurueckgeben."""
        values = []
        with open(filepath, "r", newline="") as f:
            first_line = f.readline()
            delimiter = ";" if ";" in first_line else ","
            f.seek(0)
            reader = csv.reader(f, delimiter=delimiter)
            header = next(reader)

            # Spalte finden
            col_v = 1  # Default
            for i, h in enumerate(header):
                h_lower = h.strip().lower()
                if "weg" in h_lower or h_lower == "weg_mm":
                    col_v = i
                    break

            for row in reader:
                try:
                    values.append(float(row[col_v].replace(",", ".")))
                except (ValueError, IndexError):
                    continue
        return np.array(values)

    @staticmethod
    def _rainflow_offline(signal, min_amplitude=0.1):
        """Vollstaendige Rainflow-Zaehlung (ASTM E1049, offline)."""
        # Umkehrpunkte
        if len(signal) < 3:
            return []
        tp = [signal[0]]
        for i in range(1, len(signal) - 1):
            if (signal[i] - signal[i-1]) * (signal[i+1] - signal[i]) < 0:
                tp.append(signal[i])
        tp.append(signal[-1])

        # 4-Punkt-Methode
        cycles = []
        stack = []
        for pt in tp:
            stack.append(pt)
            while len(stack) >= 4:
                s0, s1, s2, s3 = stack[-4], stack[-3], stack[-2], stack[-1]
                r_inner = abs(s1 - s2)
                if r_inner <= abs(s0 - s1) and r_inner <= abs(s2 - s3):
                    amp = r_inner / 2.0
                    del stack[-3]
                    del stack[-2]
                    if amp >= min_amplitude:
                        cycles.append((amp, (s1 + s2) / 2.0, 1.0))
                else:
                    break

        # Residuum
        for i in range(len(stack) - 1):
            amp = abs(stack[i+1] - stack[i]) / 2.0
            if amp >= min_amplitude:
                mean = (stack[i] + stack[i+1]) / 2.0
                cycles.append((amp, mean, 0.5))

        return cycles

    def _show_analysis_window(self, filepath, values, amps, counts,
                              total, max_amp):
        """Neues Toplevel-Fenster mit Rainflow-Ergebnis."""
        win = tk.Toplevel(self.root)
        win.title(f"Rainflow Analyse – {os.path.basename(filepath)}")
        win.geometry("1000x700")
        win.minsize(700, 500)

        fig = Figure(figsize=(10, 6), dpi=100)
        fig.set_facecolor("#1e1e2e")

        gs = fig.add_gridspec(1, 2, wspace=0.3,
                              left=0.08, right=0.97, top=0.90, bottom=0.12)

        # --- Links: Zeitserie ---
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.set_facecolor("#1e1e2e")
        ax1.tick_params(colors="#cdd6f4", labelsize=8)
        for spine in ax1.spines.values():
            spine.set_color("#45475a")
        ax1.grid(True, color="#45475a", alpha=0.5, linewidth=0.5)

        # Downsampling fuer Plot
        n = len(values)
        if n > 5000:
            idx = np.linspace(0, n - 1, 5000, dtype=int)
            t_plot = idx / 200.0  # Annahme 200 Hz
            v_plot = values[idx]
        else:
            t_plot = np.arange(n) / 200.0
            v_plot = values

        ax1.plot(t_plot, v_plot, color="#89b4fa", linewidth=0.5)
        ax1.set_xlabel("Zeit [s] (geschätzt)", color="#cdd6f4", fontsize=9)
        ax1.set_ylabel("Weg [mm]", color="#cdd6f4", fontsize=9)
        ax1.set_title("Messdaten", color="#cdd6f4", fontsize=11)

        # --- Rechts: Rainflow-Histogramm ---
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.set_facecolor("#1e1e2e")
        ax2.tick_params(colors="#cdd6f4", labelsize=8)
        for spine in ax2.spines.values():
            spine.set_color("#45475a")
        ax2.grid(True, color="#45475a", alpha=0.5, linewidth=0.5)

        n_bins = min(30, max(5, len(amps) // 3 + 1))
        ax2.hist(amps, bins=n_bins, weights=counts,
                 color="#fab387", alpha=0.85, edgecolor="#45475a", linewidth=0.5)
        ax2.set_xlabel("Amplitude [mm]", color="#cdd6f4", fontsize=9)
        ax2.set_ylabel("Zyklen", color="#cdd6f4", fontsize=9)
        ax2.set_title(
            f"Rainflow: {int(total)} Zyklen, max {max_amp:.3f} mm",
            color="#cdd6f4", fontsize=11
        )

        fig.suptitle(os.path.basename(filepath), color="#a6adc8", fontsize=9)

        canvas = FigureCanvasTkAgg(fig, master=win)
        canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=(8, 2))
        NavigationToolbar2Tk(canvas, win).update()

        # --- Info-Zeile ---
        info_frame = ttk.Frame(win, padding=6)
        info_frame.pack(fill="x", padx=8, pady=(0, 8))

        full_c = int(np.sum(counts[counts == 1.0]))
        half_c = int(np.sum(counts[counts == 0.5]) * 2)

        ttk.Label(info_frame, text=(
            f"Datenpunkte: {len(values):,}  |  "
            f"Vollzyklen: {full_c}  |  Halbzyklen: {half_c}  |  "
            f"Gesamt: {int(total)}  |  "
            f"Max. Amplitude: {max_amp:.4f} mm  |  "
            f"Mittlere Amplitude: {np.mean(amps):.4f} mm"
        ), foreground="gray").pack(side="left")

    # ================================================================
    # Daten loeschen
    # ================================================================
    def _clear_data(self):
        self.ring.clear()
        self.rf_acc.clear()
        self.rf_bar_container = None
        self.samples_received = 0
        self.start_time = time.time()
        self.last_rate_time = time.time()
        self.last_rate_count = 0
        self.line.set_data([], [])
        self.canvas.draw_idle()

    # ================================================================
    # Cleanup
    # ================================================================
    def on_close(self):
        self._disconnect()
        if self.logging:
            self._stop_logging()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = SerialPlotterApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
