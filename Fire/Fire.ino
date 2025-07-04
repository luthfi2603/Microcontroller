#include <ThingerESP8266.h>
#include <ESP8266WiFi.h>
#include <WifiClient.h>

#define USERNAME "winzliu"
#define DEVICE_ID "andyluthfialwin"
#define DEVICE_CREDENTIAL "+ZupmcBYTm2mN86-"

ThingerESP8266 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

#define WIFI_SSID "KOST TENTREM LOBBY"
#define WIFI_PASSWORD "meteorakalambaka18"

String kondisi = "";

#define FLAME_PIN 5
#define BUZZER_PIN 2
#define LED_PIN 4

void setup() {
    Serial.begin(115200);
    pinMode(FLAME_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    digitalWrite(BUZZER_PIN, LOW);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.println("Connected to the Wi-Fi network");

    thing.add_wifi(WIFI_SSID, WIFI_PASSWORD);

    thing["Dataku"] >> [] (pson & out) {
        out["kondisi"] = kondisi;
    };
}

unsigned long previousMilis = 0, currentMilis;
const long interval = 3000;
int api = 0;

void loop() {
    thing.handle();
    currentMilis = millis();
    if (currentMilis - previousMilis >= interval) {
        previousMilis = currentMilis;

        api = digitalRead(FLAME_PIN);
        if (api) {
            digitalWrite(BUZZER_PIN, LOW);
            digitalWrite(LED_PIN, LOW);
            kondisi = "Aman";
        } else {
            digitalWrite(BUZZER_PIN, HIGH);
            digitalWrite(LED_PIN, HIGH);
            
            kondisi = "Ada Api!";
            thing.call_endpoint("my_api_endpoint1");
        }
    }
}