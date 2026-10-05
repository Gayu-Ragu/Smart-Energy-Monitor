/*
 * ESP32 Smart Energy Monitoring System (Wokwi simulation)
 *
 * Two potentiometers simulate a voltage and a current sensor. The ESP32 reads
 * them via ADC1, scales them to engineering units, checks plausibility,
 * filters them (moving average), computes power and accumulated energy,
 * detects overload (with hysteresis), shows everything on an SSD1306 OLED and
 * sends it over Wi-Fi to a Blynk dashboard.
 *
 * NOTE: all measurements are SIMULATED, scaled potentiometer values,
 * not measurements from a real electrical circuit.
 */

#include <Arduino.h>
#include "secrets.h"          // must come BEFORE BlynkSimpleEsp32.h (defines BLYNK_TEMPLATE_ID etc.)

#define BLYNK_PRINT Serial    // Blynk debug output on the serial monitor
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================================
// Configuration
// ============================================================================

// ---- Pins ----
// Only ADC1 pins (GPIO 32-39) are used: ADC2 is unavailable while Wi-Fi is active.
constexpr uint8_t PIN_VOLTAGE_SENSE = 34;   // ADC1_CH6
constexpr uint8_t PIN_CURRENT_SENSE = 35;   // ADC1_CH7
constexpr uint8_t PIN_WARN_LED      = 26;
constexpr uint8_t PIN_RESET_BTN     = 27;   // active LOW, internal pull-up

// ---- Sensor scaling (simulated sensors) ----
// The pots span a WIDER range than the valid range, so turning a pot past the
// valid limit simulates a faulty / out-of-range sensor.
constexpr int   ADC_MAX        = 4095;      // 12-bit ADC
constexpr int   ADC_SATURATION = 4090;      // raw value at/above this = input saturated
constexpr float V_FULL_SCALE   = 300.0f;    // pot at max -> 300 V
constexpr float I_FULL_SCALE   = 12.0f;     // pot at max -> 12 A

// ---- Plausibility limits ----
constexpr float V_MAX_VALID = 260.0f;
constexpr float I_MAX_VALID = 10.0f;

// ---- Overload threshold with hysteresis ----
constexpr float POWER_LIMIT_W   = 1000.0f;  // enter OVERLOAD above this
constexpr float POWER_RELEASE_W = 950.0f;   // leave OVERLOAD below this

// ---- Simulated sensor noise (Wokwi pots are perfectly clean) ----
constexpr bool  SIMULATE_NOISE = true;
constexpr float V_NOISE_AMPL   = 2.0f;      // +/- 2 V
constexpr float I_NOISE_AMPL   = 0.05f;     // +/- 0.05 A

// ---- Task periods ----
constexpr unsigned long SAMPLE_PERIOD_MS  = 200;
constexpr unsigned long DISPLAY_PERIOD_MS = 1000;
constexpr unsigned long BLYNK_PERIOD_MS   = 2000;
constexpr unsigned long DEBOUNCE_MS       = 50;
constexpr unsigned long WIFI_TIMEOUT_MS   = 15000;

// ---- Blynk virtual pins ----
// V0 Voltage | V1 Current | V2 Power | V3 Energy (kWh) | V4 Status text | V5 Overload (0/1)

// ============================================================================
// Types
// ============================================================================

enum class SystemState : uint8_t { INIT, NORMAL, OVERLOAD, SENSOR_ERROR };

// Fixed-size moving-average filter using a ring buffer and a running sum.
template <int N>
class MovingAverage {
public:
    float update(float x) {
        sum_ -= buf_[idx_];
        buf_[idx_] = x;
        sum_ += x;
        idx_ = (idx_ + 1) % N;
        if (count_ < N) count_++;
        return sum_ / count_;   // during start-up, average only the samples we have
    }
private:
    float buf_[N] = {0};
    float sum_    = 0.0f;
    int   idx_    = 0;
    int   count_  = 0;
};

// ============================================================================
// Globals
// ============================================================================

Adafruit_SSD1306 display(128, 64, &Wire, -1);
BlynkTimer timer;

MovingAverage<5> voltageFilter;   // 5 samples x 200 ms = 1 s window
MovingAverage<5> currentFilter;

SystemState   state        = SystemState::INIT;
float         voltageV     = 0.0f;   // filtered
float         currentA     = 0.0f;   // filtered
float         powerW       = 0.0f;
double        energyWh     = 0.0;    // double: small increments added to a growing sum
unsigned long lastSampleMs = 0;
bool          oledOk       = false;

// ============================================================================
// Helpers
// ============================================================================

const char* stateName(SystemState s) {
    switch (s) {
        case SystemState::INIT:         return "INIT";
        case SystemState::NORMAL:       return "NORMAL";
        case SystemState::OVERLOAD:     return "OVERLOAD";
        case SystemState::SENSOR_ERROR: return "SENSOR_ERROR";
    }
    return "UNKNOWN";
}

float adcToValue(int raw, float fullScale) {
    return (static_cast<float>(raw) / ADC_MAX) * fullScale;
}

float addSimulatedNoise(float x, float amplitude) {
    if (!SIMULATE_NOISE) return x;
    float noise = (random(-1000, 1001) / 1000.0f) * amplitude;
    float y = x + noise;
    return (y < 0.0f) ? 0.0f : y;   // a pot at zero should not produce negative readings
}

void updateWarningLed() {
    static bool blink = false;
    switch (state) {
        case SystemState::OVERLOAD:
            digitalWrite(PIN_WARN_LED, HIGH);          // solid on
            break;
        case SystemState::SENSOR_ERROR:
            blink = !blink;
            digitalWrite(PIN_WARN_LED, blink);         // blinking
            break;
        default:
            digitalWrite(PIN_WARN_LED, LOW);
            break;
    }
}

// ============================================================================
// Periodic tasks
// ============================================================================

// Every 200 ms: acquire -> check -> filter -> compute -> classify
void sampleTask() {
    unsigned long now  = millis();
    unsigned long dtMs = now - lastSampleMs;   // unsigned math stays correct across millis() overflow
    lastSampleMs = now;

    // 1. Acquisition
    int rawV = analogRead(PIN_VOLTAGE_SENSE);
    int rawI = analogRead(PIN_CURRENT_SENSE);

    // 2. Scaling to engineering units (+ simulated noise)
    float vSample = addSimulatedNoise(adcToValue(rawV, V_FULL_SCALE), V_NOISE_AMPL);
    float iSample = addSimulatedNoise(adcToValue(rawI, I_FULL_SCALE), I_NOISE_AMPL);

    // 3. Plausibility check on UNFILTERED samples, so a fault is not averaged away
    bool sensorFault =
        (rawV >= ADC_SATURATION) || (rawI >= ADC_SATURATION) ||
        (vSample < 0.0f) || (vSample > V_MAX_VALID) ||          // < 0: defensive, for real sensors with offset
        (iSample < 0.0f) || (iSample > I_MAX_VALID);

    if (sensorFault) {
        // Invalid data: keep last good values, do NOT integrate energy
        state = SystemState::SENSOR_ERROR;
    } else {
        // 4. Filtering
        voltageV = voltageFilter.update(vSample);
        currentA = currentFilter.update(iSample);

        // 5. Power and energy (integrate P over the measured elapsed time)
        powerW    = voltageV * currentA;
        energyWh += powerW * (dtMs / 3600000.0);   // ms -> h

        // 6. Overload detection with hysteresis
        if (state == SystemState::OVERLOAD) {
            if (powerW < POWER_RELEASE_W) state = SystemState::NORMAL;
        } else {
            state = (powerW > POWER_LIMIT_W) ? SystemState::OVERLOAD : SystemState::NORMAL;
        }
    }

    updateWarningLed();

    // CSV-style log: raw vs. filtered lets you show the filter working
    Serial.printf("vRaw=%.1f,vFilt=%.1f,iRaw=%.2f,iFilt=%.2f,P=%.1f,E_Wh=%.3f,state=%s\n",
                  vSample, voltageV, iSample, currentA, powerW, energyWh, stateName(state));
}

// Every 1 s: local display
void displayTask() {
    if (!oledOk) return;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("Smart Energy Monitor"));
    display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

    display.setCursor(0, 13);
    if (state == SystemState::SENSOR_ERROR) {
        display.setTextSize(2);
        display.println(F("SENSOR"));
        display.println(F("ERROR"));
        display.setTextSize(1);
    } else {
        display.printf("V: %8.1f V\n",  voltageV);
        display.printf("I: %8.2f A\n",  currentA);
        display.printf("P: %8.1f W\n",  powerW);
        display.printf("E: %8.2f Wh\n", energyWh);
    }

    display.setCursor(0, 56);
    display.printf("%s %s", stateName(state), Blynk.connected() ? "NET:OK" : "NET:--");
    display.display();
}

// Every 2 s: cloud update
void blynkTask() {
    if (!Blynk.connected()) return;
    Blynk.virtualWrite(V0, voltageV);
    Blynk.virtualWrite(V1, currentA);
    Blynk.virtualWrite(V2, powerW);
    Blynk.virtualWrite(V3, energyWh / 1000.0);   // kWh
    Blynk.virtualWrite(V4, stateName(state));
    Blynk.virtualWrite(V5, state == SystemState::OVERLOAD ? 1 : 0);
}

// Polled from loop(): debounced push button resets the energy counter
void handleResetButton() {
    static bool          lastReading  = HIGH;
    static bool          stableState  = HIGH;
    static unsigned long lastChangeMs = 0;

    bool reading = digitalRead(PIN_RESET_BTN);
    if (reading != lastReading) {
        lastChangeMs = millis();
        lastReading  = reading;
    }
    if ((millis() - lastChangeMs) > DEBOUNCE_MS && reading != stableState) {
        stableState = reading;
        if (stableState == LOW) {          // pressed (active low)
            energyWh = 0.0;
            Serial.println(F("Energy counter reset"));
        }
    }
}

// ============================================================================
// Setup / loop
// ============================================================================

void connectWiFi() {
    Serial.printf("Connecting to Wi-Fi '%s'", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_TIMEOUT_MS) {
        delay(250);
        Serial.print('.');
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWi-Fi connected, IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println(F("\nWi-Fi not available - running in local-only mode"));
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("\n=== ESP32 Smart Energy Monitor ==="));

    pinMode(PIN_WARN_LED, OUTPUT);
    pinMode(PIN_RESET_BTN, INPUT_PULLUP);
    analogReadResolution(12);              // 0..4095 (default attenuation covers ~0..3.3 V)

    oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    if (oledOk) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println(F("Booting..."));
        display.println(F("Connecting Wi-Fi"));
        display.display();
    } else {
        Serial.println(F("OLED not found - continuing without display"));
    }

    connectWiFi();

    // Non-blocking style: the system keeps measuring even if the cloud is unreachable
    Blynk.config(BLYNK_AUTH_TOKEN);
    if (WiFi.status() == WL_CONNECTED) {
        Blynk.connect(5000);               // try for up to 5 s
    }

    lastSampleMs = millis();
    timer.setInterval(SAMPLE_PERIOD_MS,  sampleTask);
    timer.setInterval(DISPLAY_PERIOD_MS, displayTask);
    timer.setInterval(BLYNK_PERIOD_MS,   blynkTask);
}

void loop() {
    // No delay() here: everything periodic runs from BlynkTimer
    if (WiFi.status() == WL_CONNECTED) {
        Blynk.run();
    }
    timer.run();
    handleResetButton();
}
