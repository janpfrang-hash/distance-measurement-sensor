"""
Serial Plotter & Logger für ESP32 Linearpotentiometer
=====================================================
Anforderungen: pip install pyserial matplotlib

Funktionen:
- Serielle Schnittstelle erkennen und auswählen
- Live X-Y Plot (Zeit vs. Weg in mm)
- Datenlogging in CSV-Datei
- Zeitachse umschaltbar: Sekunden / Minuten / Stunden
- Zeitfenster einstellbar
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import csv
import os
from collections import deque
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure


class SerialPlotterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Poti Plotter & Logger")
        self.root.geometry("1100x720")
        self.root.minsize(800, 500)

        # --- State ---
        self.ser = None
        self.running = False
        self.read_thread = None
        self.logging = False
        self.log_file = None
        self.csv_writer = None
        self.start_time = None

        # Datenpuffer (Zeit in Sekunden, Wert in mm)
        self.max_points = 200_000  # ~16 min bei 200 Hz
        self.times = deque(maxlen=self.max_points)
        self.values = deque(maxlen=self.max_points)

        self._build_ui()
        self._setup_plot()
        self._animate()

    # ================================================================
    # UI aufbauen
    # ================================================================
    def _build_ui(self):
        # --- Top Bar: Verbindung ---
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

        # --- Mitte: Plot-Steuerung ---
        ctrl_frame = ttk.LabelFrame(self.root, text="Anzeige", padding=6)
        ctrl_frame.pack(fill="x", padx=8, pady=2)

        ttk.Label(ctrl_frame, text="Zeitachse:").pack(side="left")
        self.time_unit_var = tk.StringVar(value="s")
        for unit in ("s", "min", "h"):
            ttk.Radiobutton(ctrl_frame, text=unit, variable=self.time_unit_var, value=unit).pack(side="left", padx=2)

        ttk.Label(ctrl_frame, text="    Zeitfenster:").pack(side="left", padx=(12, 0))
        self.window_var = tk.StringVar(value="30")
        window_entry = ttk.Entry(ctrl_frame, textvariable=self.window_var, width=6)
        window_entry.pack(side="left", padx=4)
        ttk.Label(ctrl_frame, text="(in gew. Einheit, 0=alles)").pack(side="left")

        ttk.Separator(ctrl_frame, orient="vertical").pack(side="left", fill="y", padx=12)

        self.clear_btn = ttk.Button(ctrl_frame, text="Plot löschen", command=self._clear_data)
        self.clear_btn.pack(side="left", padx=4)

        # --- Logging ---
        log_frame = ttk.LabelFrame(self.root, text="Logging", padding=6)
        log_frame.pack(fill="x", padx=8, pady=2)

        self.log_btn = ttk.Button(log_frame, text="Logging starten", command=self._toggle_logging)
        self.log_btn.pack(side="left", padx=4)

        self.log_status_var = tk.StringVar(value="Kein Log aktiv")
        ttk.Label(log_frame, textvariable=self.log_status_var, foreground="gray").pack(side="left", padx=8)

        self.sample_count_var = tk.StringVar(value="Samples: 0")
        ttk.Label(log_frame, textvariable=self.sample_count_var).pack(side="right", padx=8)

        # --- Plot-Bereich ---
        self.plot_frame = ttk.Frame(self.root)
        self.plot_frame.pack(fill="both", expand=True, padx=8, pady=(2, 8))

    # ================================================================
    # Plot einrichten
    # ================================================================
    def _setup_plot(self):
        self.fig = Figure(figsize=(10, 4), dpi=100)
        self.fig.set_facecolor("#1e1e2e")
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#1e1e2e")
        self.ax.set_xlabel("Zeit [s]", color="#cdd6f4")
        self.ax.set_ylabel("Weg [mm]", color="#cdd6f4")
        self.ax.tick_params(colors="#cdd6f4")
        for spine in self.ax.spines.values():
            spine.set_color("#45475a")
        self.ax.grid(True, color="#45475a", alpha=0.5, linewidth=0.5)

        (self.line,) = self.ax.plot([], [], color="#89b4fa", linewidth=1.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        toolbar = NavigationToolbar2Tk(self.canvas, self.plot_frame)
        toolbar.update()

    # ================================================================
    # Port-Scan
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
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(0.5)  # ESP32 Reset abwarten
            self.ser.reset_input_buffer()
        except serial.SerialException as e:
            messagebox.showerror("Fehler", f"Kann Port nicht öffnen:\n{e}")
            return

        self.running = True
        self.start_time = time.time()
        self.connect_btn.config(text="Trennen")
        self.status_var.set(f"Verbunden: {port} @ {baud}")

        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def _disconnect(self):
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=2)
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.connect_btn.config(text="Verbinden")
        self.status_var.set("Getrennt")

    # ================================================================
    # Daten lesen (Thread)
    # ================================================================
    def _read_loop(self):
        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue
                    try:
                        value = float(line)
                    except ValueError:
                        continue  # Textzeilen (Info, Kalibrierung) ignorieren

                    t = time.time() - self.start_time
                    self.times.append(t)
                    self.values.append(value)

                    # Logging
                    if self.logging and self.csv_writer:
                        self.csv_writer.writerow([f"{t:.4f}", f"{value:.4f}"])
                else:
                    time.sleep(0.001)
            except (serial.SerialException, OSError):
                self.running = False
                break

    # ================================================================
    # Plot-Animation
    # ================================================================
    def _animate(self):
        if self.times:
            unit = self.time_unit_var.get()
            divisor = {"s": 1.0, "min": 60.0, "h": 3600.0}[unit]

            t_arr = [t / divisor for t in self.times]
            v_arr = list(self.values)

            # Zeitfenster
            try:
                window = float(self.window_var.get())
            except ValueError:
                window = 0

            if window > 0 and t_arr:
                t_max = t_arr[-1]
                t_min = t_max - window
                # Nur sichtbare Daten
                start_idx = 0
                for i, t in enumerate(t_arr):
                    if t >= t_min:
                        start_idx = i
                        break
                t_arr = t_arr[start_idx:]
                v_arr = v_arr[start_idx:]

            self.line.set_data(t_arr, v_arr)
            if t_arr:
                self.ax.set_xlim(t_arr[0], t_arr[-1] if t_arr[-1] > t_arr[0] else t_arr[0] + 0.1)
            if v_arr:
                vmin, vmax = min(v_arr), max(v_arr)
                margin = max(0.05, (vmax - vmin) * 0.1)
                self.ax.set_ylim(vmin - margin, vmax + margin)

            self.ax.set_xlabel(f"Zeit [{unit}]", color="#cdd6f4")
            self.canvas.draw_idle()

            self.sample_count_var.set(f"Samples: {len(self.times)}")

        self.root.after(50, self._animate)  # ~20 FPS Update

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
            self.log_file = open(filepath, "w", newline="")
            self.csv_writer = csv.writer(self.log_file, delimiter=";")
            self.csv_writer.writerow(["Zeit_s", "Weg_mm"])
            self.logging = True
            self.log_btn.config(text="Logging stoppen")
            self.log_status_var.set(f"Log: {os.path.basename(filepath)}")
        except IOError as e:
            messagebox.showerror("Fehler", f"Kann Log-Datei nicht erstellen:\n{e}")

    def _stop_logging(self):
        self.logging = False
        if self.log_file:
            self.log_file.close()
            self.log_file = None
        self.csv_writer = None
        self.log_btn.config(text="Logging starten")
        self.log_status_var.set("Log gespeichert")

    # ================================================================
    # Daten löschen
    # ================================================================
    def _clear_data(self):
        self.times.clear()
        self.values.clear()
        self.start_time = time.time()
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
