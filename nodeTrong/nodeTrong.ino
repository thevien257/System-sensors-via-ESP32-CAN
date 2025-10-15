#include <ESP32-TWAI-CAN.hpp>
#include "ESP32_C3_TimerInterrupt.h"
#include "DHT.h"
#include <Wire.h>
#include "RTClib.h"

// ==== CAN định nghĩa ====
// ESP32-C3 default pins: GPIO1=TX, GPIO0=RX
#define CAN_TX 1
#define CAN_RX 0

// ==== Cảm biến & RTC ====
#define DHTPIN 2
#define DHTTYPE DHT11
#define SDA_PIN 4
#define SCL_PIN 5

#define BUTTON 10
#define BUZZER 6

DHT dht(DHTPIN, DHTTYPE);
RTC_DS1307 rtc;

// ==== Timer phần cứng ====
ESP32Timer ITimer0(0);  // Timer0 → DHT11
ESP32Timer ITimer1(1);  // Timer1 → RTC

// ==== Chu kỳ đọc ====
#define DHT_INTERVAL_MS 5000
#define RTC_INTERVAL_MS 1000

// ==== Flags ====
bool readDHTFlag = false;
bool readRTCFlag = false;

// ==== ISR ====
bool IRAM_ATTR TimerHandler0(void* timerNo) {
  readDHTFlag = true;
  return true;
}
bool IRAM_ATTR TimerHandler1(void* timerNo) {
  readRTCFlag = true;
  return true;
}

void IRAM_ATTR handleButtonInterrupt() {
  int buttonState = digitalRead(BUTTON);
  digitalWrite(BUZZER, !buttonState);
}

// ==== TASK HANDLERS ====
void handleDHTTask() {
  readDHTFlag = false;
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  Serial.printf("Humidity: %.1f %% | Temp: %.1f °C\n", h, t);

  // Đóng gói dữ liệu DHT vào CAN (ID = 0x101)
  CanFrame txFrame = { 0 };
  txFrame.identifier = 0x03;
  txFrame.extd = 0;
  txFrame.data_length_code = 4;  // 2 float được scale nhỏ hơn

  // Ép kiểu sang int16 để truyền gọn (×10 để giữ 1 số thập phân)
  int16_t temp = (int16_t)(t * 10);
  int16_t humi = (int16_t)(h * 10);

  txFrame.data[0] = temp & 0xFF;
  txFrame.data[1] = (temp >> 8) & 0xFF;
  txFrame.data[2] = humi & 0xFF;
  txFrame.data[3] = (humi >> 8) & 0xFF;

  if (ESP32Can.writeFrame(txFrame)) {
    Serial.println("CAN Sent: Temp & Humi");
  } else {
    Serial.println("CAN Send Fail!");
  }
}

void handleRTCTask() {
  readRTCFlag = false;
  DateTime now = rtc.now();

  Serial.printf("Time: %02d:%02d:%02d  Date: %02d/%02d/%04d\n",
                now.hour(), now.minute(), now.second(),
                now.day(), now.month(), now.year());

  // Đóng gói dữ liệu RTC vào CAN (ID = 0x102)
  CanFrame txFrame = { 0 };
  txFrame.identifier = 0x01;
  txFrame.extd = 0;
  txFrame.data_length_code = 7;

  txFrame.data[0] = now.hour();
  txFrame.data[1] = now.minute();
  txFrame.data[2] = now.second();
  txFrame.data[3] = now.day();
  txFrame.data[4] = now.month();
  txFrame.data[5] = now.year() - 2000;   // chỉ gửi 2 số cuối
  txFrame.data[6] = now.dayOfTheWeek();  // 0 = Sunday, 1 = Monday...

  if (ESP32Can.writeFrame(txFrame)) {
    Serial.println("CAN Sent: RTC");
  } else {
    Serial.println("CAN Send Fail!");
  }
}

void setup() {
  Serial.begin(9600);
  delay(500);  // Cho hệ thống ổn định

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  attachInterrupt(digitalPinToInterrupt(BUTTON), handleButtonInterrupt, CHANGE);

  // ==== Init sensors ====
  dht.begin();
  Wire.begin(SDA_PIN, SCL_PIN);

  // ==== Kiểm tra RTC ====
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC, retrying...");
    Serial.println("RTC init failed, restarting...");
  }

  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(2025, 10, 15, 7, 21, 0));
  }

  // ==== Timers ====
  ITimer0.attachInterruptInterval(DHT_INTERVAL_MS * 1000, TimerHandler0);
  ITimer1.attachInterruptInterval(RTC_INTERVAL_MS * 1000, TimerHandler1);

  // ==== Init CAN (có timeout & auto-reset) ====
  if (ESP32Can.begin(TWAI_SPEED_500KBPS, CAN_TX, CAN_RX)) {

  } else {
    Serial.println("CAN bus failed to start after retries! Restarting...");
  }

  Serial.println("CAN bus started successfully!");
}


void loop() {
  if (readDHTFlag) handleDHTTask();
  if (readRTCFlag) handleRTCTask();
}
