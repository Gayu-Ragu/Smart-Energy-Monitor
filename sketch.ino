#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "secrets.h"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include<BlynkSimpleEsp32.h>
BlynkTimer timer;

uint8_t PIN_Voltage_Sense = 34;
uint8_t PIN_Current_Sense = 35;
uint8_t PIN_LED = 26;

float ADC_MAX = 4095.0f; //float forces float division in conversion
float V_full_scale = 300.0f ;
float I_full_scale = 12.0f ;

float Default_Power_Limit = 1000.0f;
float Hysteresis_W = 50.0f;
float Min_Power_limit = 100.0f;
float Max_Power_limit = 2500.0f;
float powerLimitW = 1000.0f;

unsigned long Sample_period_ms = 500;
unsigned long DISPLAY_PERIOD_MS = 1000;
unsigned long BLYNK_PERIOD_MS=2000;
unsigned long WIFI_TIMEOUT_MS=15000;

int screen_width = 128;
int screen_height = 64;
uint8_t oled_addr = 0X3C; // I2C address for almost all SSD1306 module

float V;
float I;
float power;

Adafruit_SSD1306 display (screen_width, screen_height, &Wire, -1);
// &Wire default bus on GPIO21 and 22 and -1 no extra reset pin.
bool oledok = false;

enum class SystemState : uint8_t { INIT, NORMAL, OVERLOAD };
SystemState state = SystemState::INIT;

const char* stateName(SystemState s) {
  switch (s) {
    case SystemState::INIT:     return "INIT";
    case SystemState::NORMAL:   return "NORMAL";
    case SystemState::OVERLOAD: return "OVERLOAD";
  }
  return "UNKNOWN";
}
void updateState() {
  if (state == SystemState::OVERLOAD) {
      if (power < powerLimitW - Hysteresis_W) {
      state = SystemState::NORMAL;
    }
  } 
  else {
     state = (power > powerLimitW) ? SystemState::OVERLOAD : SystemState::NORMAL;
  }
}
// the conversion
/*float adcToValue (int raw, float fullscale)
{
  return (static_cast<float>(raw)/ADC_MAX)*fullscale;
}*/
// Remeber int/int is integer then in our case it will be 0 or 1. but we need an
//exact value so we force one type into float and divide it by integer
//so the result is in float which is like 0.11,0.34.

// we can do ADC_MAX as float
float adcToValue (int raw, float fullscale)
{
  return (raw/ADC_MAX)*fullscale;
}

void Task()
{
  int raw_V = analogRead(PIN_Voltage_Sense);
  int raw_I = analogRead(PIN_Current_Sense);

  V = adcToValue(raw_V, V_full_scale);
  I = adcToValue(raw_I, I_full_scale);
  power = V*I;

  updateState();
  updateWarningLed();

  Serial.printf("raw voltage = %4d, raw Current = %4d, Voltage = %6.1f V Current=%6.1f I Power=%6.1f W | limit=%.0f W %s\n", raw_V, raw_I, V, I, power,powerLimitW, stateName(state));
}
void displayTask()
{
  if(!oledok) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0,0);
  display.println("Smart Energy Monitor");
  display.drawLine(0,10,screen_width-1,10,SSD1306_WHITE);

  display.setCursor(0,16);
  display.printf("V: %6.1f V\n",V);
  display.printf("I: %6.1f I\n",I);
  display.printf("P: %6.1f W\n",power);

  display.setCursor(0, 56);
  display.print(Blynk.connected() ? "Cloud: online" : "Cloud: offline");

  display.setCursor(0, 48);
  if (state == SystemState::OVERLOAD) {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);  
    display.print(" !! OVERLOAD !! ");
    display.setTextColor(SSD1306_WHITE);                
  } 
  else {
    display.printf("Status: %s", stateName(state));
  }

  display.display();
}

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

void blynkTask() {
  if (!Blynk.connected()) return;

  Blynk.virtualWrite(V0, V);
  Blynk.virtualWrite(V1, I);
  Blynk.virtualWrite(V2, power);
  Blynk.virtualWrite(V4, stateName(state));
  Blynk.virtualWrite(V5, state == SystemState::OVERLOAD ? 1 : 0);
}
BLYNK_WRITE(V6) {
  float requested = param.asFloat();
  powerLimitW = constrain(requested, Min_Power_limit, Max_Power_limit);
  Serial.printf("Blynk V6: received %.1f -> limit set to %.0f W\n", requested, powerLimitW);
  Serial.printf("New power limit from Blynk: %.0f W\n", powerLimitW);
}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(V6);   // ask the server for the slider's current value
}

void updateWarningLed() {
  digitalWrite(PIN_LED, state == SystemState::OVERLOAD ? HIGH : LOW);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("== Stage3: sensore => serial + OLED + Blynk==");
  analogReadResolution(12);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  oledok = display.begin(SSD1306_SWITCHCAPVCC, oled_addr);
  if (!oledok) {
    Serial.println("OLED not found - continuing without display");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Booting...");
    display.display();
  }
  connectWiFi();

  Blynk.config(BLYNK_AUTH_TOKEN);
if (WiFi.status() == WL_CONNECTED) {
  Blynk.connect(5000);   // try for up to 5 s
}

timer.setInterval(Sample_period_ms,  Task);
timer.setInterval(DISPLAY_PERIOD_MS, displayTask);
timer.setInterval(BLYNK_PERIOD_MS,   blynkTask);

}

void loop()
{
  
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.run();
  }
  timer.run();
}
