#define BLYNK_TEMPLATE_ID "TMPL6exAg2JQx"
#define BLYNK_TEMPLATE_NAME "Sensor"
#define BLYNK_AUTH_TOKEN "mZ7HW-JvWPCc1N746xWchZnknhS0J1AD"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

#define RCWL_PIN 23
#define LED_PIN_1 2
#define LED_PIN_2 4
#define LED_PIN_3 16
#define VIRTUAL_PIN_BLYNK_LED V0
#define VIRTUAL_PIN_LED_1 V1
#define VIRTUAL_PIN_LED_2 V2
#define VIRTUAL_PIN_LED_3 V3

#define WIFI_SSID "USUNETA-IOT-DLCBLT4"
#define WIFI_PASSWORD "IoTDLCBLt4?"

void setup() {
  Serial.begin(115200);
  pinMode(RCWL_PIN, INPUT);
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT);

  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);
}

BLYNK_WRITE (VIRTUAL_PIN_LED_1) {
  digitalWrite(LED_PIN_1, param.asInt());
}

BLYNK_WRITE (VIRTUAL_PIN_LED_2) {
  digitalWrite(LED_PIN_2, param.asInt());
}

BLYNK_WRITE (VIRTUAL_PIN_LED_3) {
  digitalWrite(LED_PIN_3, param.asInt());
}

int8_t motion = LOW, state = LOW;

void loop() {
  Blynk.run();
  motion = digitalRead(RCWL_PIN);

  if (motion == HIGH) { // Ada gerakan
    if (state == LOW) {
      Blynk.virtualWrite(VIRTUAL_PIN_BLYNK_LED, motion);
      Serial.println("Gerakan terdeteksi!");
      digitalWrite(LED_PIN_1, HIGH);
      state = HIGH;
    }
  } else { // Tidak ada gerakan
    if (state == HIGH) {
      Blynk.virtualWrite(VIRTUAL_PIN_BLYNK_LED, motion);
      Serial.println("Tidak ada gerakan");
      digitalWrite(LED_PIN_1, LOW);
      state = LOW;
    }
  }
}