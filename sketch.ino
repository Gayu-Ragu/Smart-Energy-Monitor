#include <Arduino.h>
#include "secrets.h"            // must come BEFORE the Blynk include

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---- Pins (ADC1 only for analog inputs) ----
constexpr uint8_t PIN_Voltage_Sense = 34;
constexpr uint8_t PIN_Current_Sense = 35;
constexpr uint8_t PIN_LED           = 26;

// ---- Scaling: 0..4095 -> engineering units ----
constexpr float ADC_MAX      = 4095.0f;   // float forces float division in adcToValue()
constexpr float V_full_scale = 300.0f;
constexpr float I_full_scale = 12.0f;

// ---- Simulated sensor noise (Wokwi pots are perfectly clean) ----
constexpr bool  SIMULATE_NOISE = true;
constexpr float V_NOISE_AMPL   = 2.0f;    // +/- 2 V
constexpr float I_NOISE_AMPL   = 0.05f;   // +/- 0.05 A

// ---- Overload detection ----
constexpr float Default_Power_Limit = 1000.0f;
constexpr float Hysteresis_W        = 50.0f;
constexpr float Min_Power_limit     = 100.0f;
constexpr float Max_Power_limit     = 2500.0f;
float powerLimitW = Default_Power_Limit;   // changed from Blynk (V6)

// ---- Timing ----
constexpr unsigned long Sample_period_ms  = 200;    // 5 samples x 200 ms = 1 s filter window
constexpr unsigned long DISPLAY_PERIOD_MS = 1000;
constexpr unsigned long BLYNK_PERIOD_MS   = 2000;
constexpr unsigned long WIFI_TIMEOUT_MS   = 15000;
BlynkTimer timer;

// ---- OLED ----
constexpr int     screen_width  = 128;
constexpr int     screen_height = 64;
constexpr uint8_t oled_addr     = 0x3C;
Adafruit_SSD1306 display(screen_width, screen_height, &Wire, -1);
bool oledok = false;

// ---- Moving-average filter ----
template <int N>
class MovingAverage {
public:
  float update(float x) {
    sum_ -= buf_[idx_];        // remove the oldest sample from the sum
    buf_[idx_] = x;            // overwrite it with the new sample
    sum_ += x;                 // add the new sample to the sum
    idx_ = (idx_ + 1) % N;     // next slot, wrap around at N
    if (count_ < N) count_++;
    return sum_ / count_;      // at startup: average only the samples we have
  }

private:
  float buf_[N] = {0};
  float sum_    = 0.0f;
  int   idx_    = 0;
  int   count_  = 0;
};

MovingAverage<5> voltageFilter;
MovingAverage<5> currentFilter;

// ---- System state ----
enum class SystemState : uint8_t { INIT, NORMAL, OVERLOAD };
SystemState state = SystemState::INIT;

// ---- Measurements (shared between tasks) ----
float  V        = 0.0f;   // filtered voltage [V]
float  I        = 0.0f;   // filtered current [A]
float  power    = 0.0f;   // [W]
double energyWh = 0.0;    // double: tiny increments added to a growing total
unsigned long lastSampleMs = 0;

// ============================================================================
// Helpers
// ============================================================================

const char* stateName(SystemState s) {
  switch (s) {
    case SystemState::INIT:     return "INIT";
    case SystemState::NORMAL:   return "NORMAL";
    case SystemState::OVERLOAD: return "OVERLOAD";
  }
  return "UNKNOWN";
}

float adcToValue(int raw, float fullscale) {
  return (raw / ADC_MAX) * fullscale;
}

float addSimulatedNoise(float x, float amplitude) {
  if (!SIMULATE_NOISE) return x;
  float noise = (random(-1000, 1001) / 1000.0f) * amplitude;
  float y = x + noise;
  return (y < 0.0f) ? 0.0f : y;   // a pot at zero should not give negative readings
}

void updateState() {
  if (state == SystemState::OVERLOAD) {
    // Already in overload: only leave when clearly below the limit
    if (power < powerLimitW - Hysteresis_W) {
      state = SystemState::NORMAL;
    }
  } else {
    // Not in overload: enter as soon as the limit is exceeded
    state = (power > powerLimitW) ? SystemState::OVERLOAD : SystemState::NORMAL;
  }
}

void updateWarningLed() {
  digitalWrite(PIN_LED, state == SystemState::OVERLOAD ? HIGH : LOW);
}

// ============================================================================
// Tasks
// ============================================================================

void Task() {
  // 1. Real elapsed time since the last sample
  unsigned long now  = millis();
  unsigned long dtMs = now - lastSampleMs;
  lastSampleMs = now;

  // 2. Acquisition
  int raw_V = analogRead(PIN_Voltage_Sense);
  int raw_I = analogRead(PIN_Current_Sense);

  // 3. Scaling + simulated noise
  float vSample = addSimulatedNoise(adcToValue(raw_V, V_full_scale), V_NOISE_AMPL);
  float iSample = addSimulatedNoise(adcToValue(raw_I, I_full_scale), I_NOISE_AMPL);

  // 4. Filtering
  V = voltageFilter.update(vSample);
  I = currentFilter.update(iSample);

  // 5. Power and energy
  power     = V * I;
  energyWh += power * (dtMs / 3600000.0);   // ms -> hours

  // 6. Decide and act
  updateState();
  updateWarningLed();

  Serial.printf("V_raw=%6.1f V_filt=%6.1f | I_raw=%5.2f I_filt=%5.2f | P=%7.1f W  E=%8.3f Wh | limit=%.0f W %s\n",
                vSample, V, iSample, I, power, energyWh, powerLimitW, stateName(state));
}

void displayTask() {
  if (!oledok) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("Smart Energy Monitor");
  display.drawLine(0, 10, screen_width - 1, 10, SSD1306_WHITE);

  display.setCursor(0, 16);
  display.printf("V: %8.1f V\n",  V);
  display.printf("I: %8.2f A\n",  I);
  display.printf("P: %8.1f W\n",  power);
  display.printf("E: %8.2f Wh\n", energyWh);

  display.setCursor(0, 48);
  if (state == SystemState::OVERLOAD) {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);   // inverted
    display.print(" !! OVERLOAD !! ");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.printf("Status: %s", stateName(state));
  }

  display.setCursor(0, 56);
  display.print(Blynk.connected() ? "Cloud: online" : "Cloud: offline");

  display.display();
}

void blynkTask() {
  if (!Blynk.connected()) return;

  Blynk.virtualWrite(V0, V);
  Blynk.virtualWrite(V1, I);
  Blynk.virtualWrite(V2, power);
  Blynk.virtualWrite(V3, energyWh / 1000.0);   // kWh
  Blynk.virtualWrite(V4, stateName(state));
  Blynk.virtualWrite(V5, state == SystemState::OVERLOAD ? 1 : 0);
}

// ============================================================================
// Blynk callbacks (at file level, outside any function)
// ============================================================================

BLYNK_WRITE(V6) {
  float requested = param.asFloat();
  powerLimitW = constrain(requested, Min_Power_limit, Max_Power_limit);
  Serial.printf("Blynk V6: received %.1f -> limit set to %.0f W\n", requested, powerLimitW);
}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(V6);   // get the slider's current value after (re)connecting
}

// ============================================================================
// Setup / loop
// ============================================================================

void connectWiFi() {
  Serial.printf("Connecting to Wi-Fi '%s'", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT_MS) {
    delay(250);
    Serial.print('.');
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWi-Fi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWi-Fi not available - running in local-only mode");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Stage 5: + filter + energy ===");
  analogReadResolution(12);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);   // defined state at startup

  oledok = display.begin(SSD1306_SWITCHCAPVCC, oled_addr);
  if (!oledok) {
    Serial.println("OLED not found - continuing without display");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Booting...");
    display.println("Connecting Wi-Fi");
    display.display();
  }

  connectWiFi();

  Blynk.config(BLYNK_AUTH_TOKEN);
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.connect(5000);   // try for up to 5 s
  }

  lastSampleMs = millis();   // start energy integration NOW, not at boot
  timer.setInterval(Sample_period_ms,  Task);
  timer.setInterval(DISPLAY_PERIOD_MS, displayTask);
  timer.setInterval(BLYNK_PERIOD_MS,   blynkTask);
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.run();
  }
  timer.run();
}