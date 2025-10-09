#include <ESP32-TWAI-CAN.hpp>
#include <SoftwareSerial.h>
#include <Nextion.h>

SoftwareSerial nextionSerial(18, 19);  // RX, TX

// ==== Nextion components ====
NexSlider sSlider = NexSlider(0, 3, "sSlider");
NexText tTemp = NexText(0, 8, "tTemp");
NexText tHumi = NexText(0, 7, "tHumi");
NexText tDis = NexText(0, 11, "tDis");
NexText tTime = NexText(0, 1, "tTime");
NexText tDate = NexText(0, 2, "tDate");

// ==== CAN config ====
#define CAN_TX 1
#define CAN_RX 0

// ==== Motor Driver pins ====
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 12;

#define ENA 4
#define IN1 5
#define IN2 6

const char *daysOfWeek[7] = {
  "Sunday", "Monday", "Tuesday", "Wednesday",
  "Thursday", "Friday", "Saturday"
};

// CAN RX Interrupt callback - processes everything immediately
void onCANReceive(const CanFrame &frame) {
  // DHT11 Temperature & Humidity (ID = 0x101)
  if (frame.identifier == 0x101 && frame.data_length_code == 4) {
    int16_t temp_raw = frame.data[0] | (frame.data[1] << 8);
    int16_t humi_raw = frame.data[2] | (frame.data[3] << 8);
    float temp = temp_raw / 10.0;
    float humi = humi_raw / 10.0;

    // Serial.printf(">> DHT11: Temp = %.1f °C | Humi = %.1f %%\n", temp, humi);

    char buf[64];
    sprintf(buf, "%.1fC", temp);
    tTemp.setText(buf);

    sprintf(buf, "%.1f%%", humi);
    tHumi.setText(buf);
  }

  else if (frame.identifier == 0x01 && frame.data_length_code == 7) {
    uint8_t hour = frame.data[0];
    uint8_t minute = frame.data[1];
    uint8_t second = frame.data[2];
    uint8_t day = frame.data[3];
    uint8_t month = frame.data[4];
    uint16_t year = 2000 + frame.data[5];
    uint8_t dow = frame.data[6];

    // Serial.printf(">> RTC: %s %02d:%02d:%02d  %02d/%02d/%04d\n",
    //               daysOfWeek[dow], hour, minute, second, day, month, year);

    char buf[64];
    sprintf(buf, "%02d:%02d:%02d", hour, minute, second);
    tTime.setText(buf);

    sprintf(buf, "%s %02d/%02d/%04d", daysOfWeek[dow], day, month, year);
    tDate.setText(buf);
  }

  // Ultrasonic Distance (ID = 0x103)
  else if (frame.identifier == 0x103 && frame.data_length_code == 1) {
    uint16_t distance = frame.data[0];
    // Serial.printf(">> Ultrasonic Distance = %d cm\n", distance);

    char buf[64];
    sprintf(buf, "%dcm", distance);
    tDis.setText(buf);
  }
}

void setup() {
  Serial.begin(115200);
  nextionSerial.begin(9600);
  nexInit();
  delay(1000);

  // ==== Motor setup ====
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL);

  Serial.println("CAN Receiver with Interrupts");

  // Set CAN pins
  ESP32Can.setPins(CAN_TX, CAN_RX);

  // Start CAN bus
  if (ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX)) {
    Serial.println("CAN bus started successfully!");

    // Register interrupt callback
    ESP32Can.onReceive(onCANReceive);

  } else {
    Serial.println("CAN bus failed to start!");
    while (1)
      ;
  }
}

void loop() {

  uint32_t currentSliderValue = 0;

  if (sSlider.getValue(&currentSliderValue)) {
    ledcWrite(PWM_CHANNEL, currentSliderValue);
  }

  delay(10);
}