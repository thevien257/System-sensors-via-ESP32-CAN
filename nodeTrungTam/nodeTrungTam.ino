#include <ESP32-TWAI-CAN.hpp>
#include <Nextion.h>
#include <SoftwareSerial.h>

SoftwareSerial nextionSerial(18, 19);

NexText tTime = NexText(0, 1, "tTime");
NexText tDate = NexText(0, 2, "tDate");
NexText tDis = NexText(0, 11, "tDis");
NexText tTemp = NexText(0, 8, "tTemp");
NexText tHumi = NexText(0, 7, "tHumi");
NexSlider sSlider = NexSlider(0, 3, "sSlider");

const int ENA = 4;
const int IN1 = 5;
const int IN2 = 6;

const int CAN_TX = 1;
const int CAN_RX = 0;

uint32_t currentSliderVal;

const char
  *daysOfWeek[7] = {
    "Sunday", "Monday", "Tuesday", "Wednesday",
    "Thursday", "Friday", "Saturday"
  };

void CAN_RECEIVE(const CanFrame &rxFrame) {
  char buff[64];
  if (rxFrame.identifier == 0x01 && rxFrame.data_length_code == 7) {
    uint8_t hour = rxFrame.data[0];
    uint8_t min = rxFrame.data[1];
    uint8_t sec = rxFrame.data[2];
    uint8_t day = rxFrame.data[3];
    uint8_t month = rxFrame.data[4];
    uint16_t year = 2000 + rxFrame.data[5];
    uint8_t dow = rxFrame.data[6];

    /*%d = format as a decimal integer
        0 = pad with zeros (instead of spaces)
        2 = minimum width of 2 characters*/
    sprintf(buff, "%02d:%02d:%02d", hour, min, sec);
    tTime.setText(buff);

    sprintf(buff, "%s/%02d/%02d/%04d", daysOfWeek[dow], day, month, year);
    tDate.setText(buff);
  }
  if (rxFrame.identifier == 0x02 && rxFrame.data_length_code == 2) {
    uint16_t dis = rxFrame.data[0] | (rxFrame.data[1] << 8);
    sprintf(buff, "%dcm", dis);
    tDis.setText(buff);
  }

  if (rxFrame.identifier == 0x03 && rxFrame.data_length_code == 4) {
    uint16_t temp = rxFrame.data[0] | (rxFrame.data[1] << 8);
    uint16_t humi = rxFrame.data[2] | (rxFrame.data[3] << 8);

    float tempF = temp / 10.0;
    sprintf(buff, "%.1fC", tempF);
    tTemp.setText(buff);

    float humiF = humi / 10.0;
    sprintf(buff, "%.1f%%", humiF);
    tHumi.setText(buff);
  }
}

void setup() {
  Serial.begin(9600);
  nextionSerial.begin(9600);
  nexInit();

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  if (ESP32Can.begin(TWAI_SPEED_500KBPS, CAN_TX, CAN_RX)) {
    Serial.println("Init OK");
    ESP32Can.onReceive(&CAN_RECEIVE);
  } else {
    Serial.println("Init ERROR");
    while (1)
      ;
  }
}

void loop() {
  if (sSlider.getValue(&currentSliderVal)) {
    analogWrite(ENA, currentSliderVal);
  }
}