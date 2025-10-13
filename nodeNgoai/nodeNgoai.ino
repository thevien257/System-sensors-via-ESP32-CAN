#include "ESP32_C3_TimerInterrupt.h"
#include <NewPing.h>
#include <ESP32-TWAI-CAN.hpp>  // Thư viện CAN

// ==== Khai báo chân kết nối ====
#define LDR 0
#define LED 1
#define TRIG 2
#define ECHO 3
#define MAX_DISTANCE 200  // cm

// ==== Đối tượng cảm biến ====
NewPing sonar(TRIG, ECHO, MAX_DISTANCE);

hw_timer_t* Timer0_Cfg = NULL;

// ==== Timer phần cứng cho LDR ====
ESP32Timer ITimer0(0);  // Timer0 → LDR
ESP32Timer ITimer1(1);  // Timer1 cho Ultrasonic

// ==== Chu kỳ đọc ====
#define LDR_INTERVAL_MS 200
#define US_INTERVAL_MS 100

// ==== Flags ====
volatile bool readLDRFlag = false;
volatile bool readUSFlag = false;

// ==== CAN Config ====
#define CAN_TX 6
#define CAN_RX 7

// ==== ISR ====
void IRAM_ATTR TimerHandler0() {
  readLDRFlag = true;
}
bool IRAM_ATTR TimerHandler1(void* timerNo) {
  readUSFlag = true;
  return true;  // phải trả về true cho ESP32TimerInterrupt
}

// ==== TASK HANDLERS ====
void handleLDRTask() {
  readLDRFlag = false;
  uint16_t ldr_val = analogRead(LDR);

  if (ldr_val > 3000) digitalWrite(LED, HIGH);
  else digitalWrite(LED, LOW);

  Serial.print("LDR: ");
  Serial.println(ldr_val);
}

void handleUltrasonicTask() {
  readUSFlag = false;
  unsigned int distance = sonar.ping_cm();

  Serial.print("Distance: ");
  if (distance == 0) {
    Serial.println("Out of range");
  } else {
    Serial.print(distance);
    Serial.println(" cm");
  }

  // Gửi Distance qua CAN (ID = 0x02)
  CanFrame txFrame = { 0 };
  txFrame.identifier = 0x02;
  txFrame.extd = 0;
  txFrame.data_length_code = 1;
  txFrame.data[0] = distance;
  // txFrame.data[1] = (distance >> 8) & 0xFF;

  if (ESP32Can.writeFrame(txFrame)) {
    Serial.printf("Sent Distance = %d cm via CAN (ID: 0x02)\n", distance);
  } else {
    Serial.println("Failed to send Distance via CAN!");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);

  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &TimerHandler0, true);
  timerAlarmWrite(Timer0_Cfg, LDR_INTERVAL_MS * 1000, true);
  timerAlarmEnable(Timer0_Cfg);

  // Bật Timer0 cho LDR
  // if (ITimer0.attachInterruptInterval(LDR_INTERVAL_MS * 1000, TimerHandler0)) {
  //   Serial.println("Timer0 (LDR) started");
  // }

  // Bật Timer1 cho Ultrasonic
  if (ITimer1.attachInterruptInterval(US_INTERVAL_MS * 1000, TimerHandler1)) {
    Serial.println("Timer1 (Ultrasonic) started");
  }

  // ==== CAN init có retry và reset ====
  ESP32Can.setPins(CAN_TX, CAN_RX);
  delay(200);

  if (ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX)) {
    Serial.println("CAN bus started successfully!");
  } else {
    Serial.println("CAN bus failed to start after retries! Restarting...");
  }
}

void loop() {
  if (readLDRFlag) handleLDRTask();
  if (readUSFlag) handleUltrasonicTask();
}
