/*
 * ESP32 + ADS1115 - Linearpotentiometer Wegmessung
 * =================================================
 * Potentiometer: 75 mm Hub, Messbereich 0-10 mm
 * ADS1115: 16-Bit ADC, I2C, 860 SPS Continuous Mode
 * Ausgabe: Serieller Plotter (Arduino IDE)
 *
 * Verdrahtung:
 *   ADS1115 SCL  -> ESP32 GPIO22
 *   ADS1115 SDA  -> ESP32 GPIO23
 *   ADS1115 ADDR -> GND (Adresse 0x48)
 *   ADS1115 VDD  -> 3.3V
 *   ADS1115 GND  -> GND
 *   Poti Schleifer -> ADS1115 A0
 *   Poti Enden     -> 3.3V und GND
 *
 * Kalibrierung:
 *   Sende 'C' ueber Serial Monitor um Kalibrierung zu starten.
 *   Schritt 1: Poti auf 0 mm Position -> Enter
 *   Schritt 2: Poti auf 10 mm Position -> Enter
 *   Kalibrierung wird im NVS (Flash) gespeichert.
 */

#include <Wire.h>
#include <Preferences.h>

// ============================================================
// Konfiguration
// ============================================================
#define I2C_SDA           23
#define I2C_SCL           22
#define ADS1115_ADDR      0x48

// ADS1115 Register
#define ADS_REG_CONVERSION 0x00
#define ADS_REG_CONFIG     0x01

// ADS1115 Config Bits
// OS: Start single / in continuous mode: no effect when writing
// MUX: AIN0 vs GND (single-ended)
// PGA: +/- 4.096V (Gain 1) -> guter Bereich fuer 3.3V Poti
// MODE: Continuous
// DR: 860 SPS
// COMP_QUE: Disable comparator
//
// Bit 15:    OS    = 1 (start)
// Bit 14-12: MUX   = 100 (AIN0-GND)
// Bit 11-9:  PGA   = 001 (+/- 4.096V)
// Bit 8:     MODE  = 0 (continuous)
// Bit 7-5:   DR    = 111 (860 SPS)
// Bit 4:     COMP_MODE = 0
// Bit 3:     COMP_POL  = 0
// Bit 2:     COMP_LAT  = 0
// Bit 1-0:   COMP_QUE  = 11 (disable)
//
// = 0b 1_100_001_0_111_0_0_0_11
// = 0xC2E3

#define ADS_CONFIG_VALUE  0xC2E3

// Messparameter
#define SAMPLE_RATE_HZ    200
#define SERIAL_BAUD       115200

// Gleitender Mittelwert (reduziert Rauschen, erhaelt Geschwindigkeit)
#define FILTER_SIZE       4

// ============================================================
// Globale Variablen
// ============================================================
Preferences prefs;

// Kalibrierung: ADC-Rohwerte an den beiden Kalibrierpunkten
int32_t cal_raw_0mm  = 0;       // ADC-Wert bei 0 mm
int32_t cal_raw_10mm = 10000;   // ADC-Wert bei 10 mm (Startwerte)

// Ringpuffer fuer gleitenden Mittelwert
int16_t filterBuf[FILTER_SIZE];
uint8_t filterIdx = 0;
bool    filterFilled = false;

// Timing
uint32_t sampleIntervalUs = 1000000UL / SAMPLE_RATE_HZ;
uint32_t lastSampleUs = 0;

// Kalibrierungsmodus
bool calibrating = false;
uint8_t calStep = 0;

// ============================================================
// ADS1115 Low-Level Funktionen
// ============================================================

void ads_writeRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  Wire.endTransmission();
}

int16_t ads_readRegister(uint8_t reg) {
  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ADS1115_ADDR, (uint8_t)2);
  int16_t result = ((int16_t)Wire.read()) << 8;
  result |= Wire.read();
  return result;
}

bool ads_conversionReady() {
  uint16_t config = (uint16_t)ads_readRegister(ADS_REG_CONFIG);
  return (config & 0x8000) != 0;  // OS bit = 1 wenn fertig
}

void ads_init() {
  ads_writeRegister(ADS_REG_CONFIG, ADS_CONFIG_VALUE);
  delay(10);
}

int16_t ads_readRaw() {
  return ads_readRegister(ADS_REG_CONVERSION);
}

// ============================================================
// Filter
// ============================================================

int16_t filterUpdate(int16_t newVal) {
  filterBuf[filterIdx] = newVal;
  filterIdx = (filterIdx + 1) % FILTER_SIZE;
  if (filterIdx == 0) filterFilled = true;

  uint8_t count = filterFilled ? FILTER_SIZE : filterIdx;
  int32_t sum = 0;
  for (uint8_t i = 0; i < count; i++) {
    sum += filterBuf[i];
  }
  return (int16_t)(sum / count);
}

// ============================================================
// Kalibrierung laden / speichern
// ============================================================

void loadCalibration() {
  prefs.begin("poti_cal", true);  // read-only
  cal_raw_0mm  = prefs.getInt("raw_0mm",  0);
  cal_raw_10mm = prefs.getInt("raw_10mm", 10000);
  prefs.end();

  // Plausibilitaet pruefen
  if (cal_raw_0mm == cal_raw_10mm) {
    cal_raw_0mm  = 0;
    cal_raw_10mm = 10000;
  }
}

void saveCalibration() {
  prefs.begin("poti_cal", false);  // read-write
  prefs.putInt("raw_0mm",  (int32_t)cal_raw_0mm);
  prefs.putInt("raw_10mm", (int32_t)cal_raw_10mm);
  prefs.end();
}

// ============================================================
// Umrechnung RAW -> mm
// ============================================================

float rawToMm(int16_t raw) {
  // Lineare Interpolation zwischen den Kalibrierpunkten
  float mm = 10.0f * (float)(raw - cal_raw_0mm) / (float)(cal_raw_10mm - cal_raw_0mm);
  return mm;
}

// ============================================================
// Kalibrierungsroutine (interaktiv ueber Serial)
// ============================================================

void startCalibration() {
  calibrating = true;
  calStep = 0;
  Serial.println();
  Serial.println("=== KALIBRIERUNG ===");
  Serial.println("Schritt 1: Potentiometer auf 0 mm Position bringen.");
  Serial.println("Dann beliebige Taste + Enter druecken.");
}

// Mehrere Samples mitteln fuer stabilen Kalibrierwert
// Zeitbasiert (kein Ready-Bit-Polling, da im Continuous Mode unzuverlaessig)
int32_t readCalibrationAverage(uint16_t numSamples) {
  int64_t sum = 0;
  for (uint16_t i = 0; i < numSamples; i++) {
    delay(2);  // ~2 ms > 1/860 SPS -> sicher neuer Wert
    sum += ads_readRaw();
  }
  return (int32_t)(sum / numSamples);
}

void handleCalibrationInput() {
  if (!Serial.available()) return;

  // Eingabe konsumieren
  while (Serial.available()) Serial.read();

  if (calStep == 0) {
    // 0 mm Punkt aufnehmen
    Serial.print("Messe 0 mm Punkt (256 Samples)... ");
    cal_raw_0mm = readCalibrationAverage(256);
    Serial.print("Raw = ");
    Serial.println(cal_raw_0mm);
    Serial.println();
    Serial.println("Schritt 2: Potentiometer auf 10 mm Position bringen.");
    Serial.println("Dann beliebige Taste + Enter druecken.");
    calStep = 1;
  }
  else if (calStep == 1) {
    // 10 mm Punkt aufnehmen
    Serial.print("Messe 10 mm Punkt (256 Samples)... ");
    cal_raw_10mm = readCalibrationAverage(256);
    Serial.print("Raw = ");
    Serial.println(cal_raw_10mm);

    // Validierung
    if (abs(cal_raw_10mm - cal_raw_0mm) < 100) {
      Serial.println("FEHLER: Zu wenig Unterschied zwischen 0mm und 10mm!");
      Serial.println("Kalibrierung abgebrochen. Bitte erneut mit 'C' starten.");
    } else {
      saveCalibration();
      Serial.println();
      Serial.println("Kalibrierung gespeichert!");
      Serial.print("  0 mm -> Raw: ");
      Serial.println(cal_raw_0mm);
      Serial.print(" 10 mm -> Raw: ");
      Serial.println(cal_raw_10mm);
      Serial.print(" Aufloesung: ~");
      float resolution_um = 10000.0f / abs(cal_raw_10mm - cal_raw_0mm);
      Serial.print(resolution_um, 2);
      Serial.println(" um/Digit");
    }

    calibrating = false;
    calStep = 0;
    Serial.println("=== Messung laeuft ===");
    Serial.println();
  }
}

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println("ESP32 + ADS1115 Linearpotentiometer");
  Serial.println("====================================");
  Serial.println("Sende 'C' fuer Kalibrierung");
  Serial.println("Sende 'I' fuer Info");
  Serial.println();

  // I2C initialisieren (400 kHz fuer schnelle Kommunikation)
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // ADS1115 pruefen
  Wire.beginTransmission(ADS1115_ADDR);
  uint8_t error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("FEHLER: ADS1115 nicht gefunden auf 0x48!");
    Serial.println("Verkabelung pruefen: SDA=GPIO23, SCL=GPIO22");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("ADS1115 gefunden.");

  // ADS1115 konfigurieren: Continuous Mode, 860 SPS
  ads_init();

  // Kalibrierung laden
  loadCalibration();
  Serial.print("Kalibrierung geladen: 0mm=");
  Serial.print(cal_raw_0mm);
  Serial.print("  10mm=");
  Serial.println(cal_raw_10mm);
  Serial.println();

  // Filter initialisieren
  for (uint8_t i = 0; i < FILTER_SIZE; i++) filterBuf[i] = 0;

  // Erstes Sample abwarten
  delay(5);
  lastSampleUs = micros();

  // Plotter-Header (Arduino IDE Serial Plotter)
  Serial.println("Weg_mm");
}

// ============================================================
// Loop
// ============================================================

void loop() {
  // Kalibrierungsmodus
  if (calibrating) {
    handleCalibrationInput();
    return;
  }

  // Befehle pruefen
  if (Serial.available()) {
    char c = Serial.read();
    // Rest der Zeile konsumieren
    while (Serial.available()) Serial.read();

    if (c == 'C' || c == 'c') {
      startCalibration();
      return;
    }
    else if (c == 'I' || c == 'i') {
      Serial.println();
      Serial.println("=== INFO ===");
      Serial.print("Samplerate: ");
      Serial.print(SAMPLE_RATE_HZ);
      Serial.println(" Hz");
      Serial.print("Filter: Gleitender Mittelwert, N=");
      Serial.println(FILTER_SIZE);
      Serial.print("Kalibr. 0mm Raw:  ");
      Serial.println(cal_raw_0mm);
      Serial.print("Kalibr. 10mm Raw: ");
      Serial.println(cal_raw_10mm);
      float res = 10000.0f / abs(cal_raw_10mm - cal_raw_0mm);
      Serial.print("Aufloesung: ~");
      Serial.print(res, 2);
      Serial.println(" um/Digit");
      Serial.println("============");
      Serial.println();
      return;
    }
  }

  // Sample-Timing (200 Hz)
  uint32_t nowUs = micros();
  if ((nowUs - lastSampleUs) >= sampleIntervalUs) {
    lastSampleUs += sampleIntervalUs;

    // Wenn wir zu weit hinterher sind, aufholen
    if ((nowUs - lastSampleUs) > sampleIntervalUs * 2) {
      lastSampleUs = nowUs;
    }

    // ADC lesen
    int16_t raw = ads_readRaw();

    // Filtern
    int16_t filtered = filterUpdate(raw);

    // In mm umrechnen
    float mm = rawToMm(filtered);

    // Ausgabe fuer Arduino Serial Plotter
    // Format: Ein Wert pro Zeile = eine Kurve im Plotter
    Serial.println(mm, 4);
  }
}

