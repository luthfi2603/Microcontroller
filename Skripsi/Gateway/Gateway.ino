#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "secrets.h"

// Deklarasi variabel global
constexpr const char* MQTT_TOPIC_GATEWAY_TELE_PUB = "v1/gateway/telemetry";
WiFiClient wiFiClient;
WiFiClientSecure secureWiFiClient;
PubSubClient mqttClient(wiFiClient);

// Prototipe fungsi
void urlEncode(const char *str, char *encodedStr, size_t maxLen);
void sendMessageToTelegram(const char *message);
void mqttReconnect();

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Menghubungkan ke jaringan Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print(F("\r\nConnecting to Wi-Fi"));
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(F("."));

    delay(500);
  }
  Serial.println(F("\r\nConnected to the Wi-Fi network"));

  secureWiFiClient.setInsecure(); // Bypass verifikasi sertifikat SSL

  // Menghubungkan ke MQTT Broker
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);

  while (!mqttClient.connected()) {
    Serial.println(F("----------------\r\nConnecting to MQTT Broker..."));
    if (mqttClient.connect("LandSlidEtect_Gateway", GATEWAY_ACCESS_TOKEN, "")) {
      Serial.println(F("Connected to MQTT Broker"));
    } else {
      Serial.print(F("Failed with state "));
      Serial.print(mqttClient.state());
      Serial.println(F(" try again in 5 seconds"));

      delay(5000);
    }
  }

  sendMessageToTelegram("Halo dari ESP32!\nフリーナちゃんとビビアンくん");
}

void loop() {
  uint32_t currentTime = millis();
  static uint32_t lastMqttReconnectTime = 0, lastPublishTime = 0;

  if (!mqttClient.connected()) {
    if (currentTime - lastMqttReconnectTime >= 5000) {
      lastMqttReconnectTime = currentTime;

      mqttReconnect();
    }
  } else {
    mqttClient.loop();
  }

  if (mqttClient.connected() && (currentTime - lastPublishTime >= 2000)) {
    lastPublishTime = currentTime;

    JsonDocument json;

    /* // Parsing JSON string
    DeserializationError error = deserializeJson(json, loRaPayload);

    if (error) {
      Serial.print(F("Failed to parsing JSON LoRa"));
      Serial.println(error.f_str());
    }

    float nilaiAx = json["ax"];
    float nilaiAy = json["ay"];
    const char* statusGempa = json["status_gempa"]; */

    json["Gateway"][0]["ax"] = 0.5;
    json["Node 1"][0]["ax"]  = 0.6;
    json["Node 2"][0]["ax"]  = 0.7;

    char payload[128];
    serializeJson(json, payload);
    
    Serial.println(F("----------------\r\nPublish:"));
    Serial.println(payload);
    mqttClient.publish(MQTT_TOPIC_GATEWAY_TELE_PUB, payload);
  }
}

void urlEncode(const char *str, char *encodedStr, size_t maxLen) {
  size_t encodedIdx = 0;

  for (size_t i = 0; i < strlen(str); i++) {
    // Cegah buffer overflow dengan pastikan wadah masih muat untuk 3 karakter ("%XX") + 1 karakter penutup ('\0')
    if (encodedIdx + 3 >= maxLen - 1) {
      Serial.println(F("Buffer overflow, string cut"));
      break; 
    }

    uint8_t c = (uint8_t)str[i];

    // Menurut RFC 3986, huruf, angka, dan 4 simbol ini tidak boleh di-encode
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      encodedStr[encodedIdx++] = c;
    } else { // Sisanya (termasuk spasi dan simbol unik), ubah ke format %HEX
      snprintf(&encodedStr[encodedIdx], 4, "%%%02X", c);
      encodedIdx += 3;
    }
  }

  // Null Terminator
  encodedStr[encodedIdx] = '\0';
}

void sendMessageToTelegram(const char *message) {
  char encodedMsg[256];
  urlEncode(message, encodedMsg, sizeof(encodedMsg));

  char apiUrl[512];
  snprintf(apiUrl, sizeof(apiUrl),
           "https://api.telegram.org/bot%s/sendMessage?chat_id=%s&text=%s",
           TELEGRAM_BOT_API_TOKEN, CHAT_ID, encodedMsg);
  
  Serial.println(F("----------------\r\nTrying request to: "));
  Serial.println(apiUrl);

  HTTPClient http;
  http.begin(secureWiFiClient, apiUrl);

  int16_t httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    Serial.print(F("HTTP Response Code: "));
    Serial.println(httpResponseCode);

    // (Catatan: http.getString() masih menggunakan String, tapi tidak apa-apa untuk 
    // respons sekali lewat. Jika ingin 100% C-String, bisa pakai http.getStream())
    String payload = http.getString();
    Serial.println(F("Server Response: "));
    Serial.println(payload);
  } else {
    Serial.print(F("Error Code: "));
    Serial.println(httpResponseCode);
  }

  http.end();
}

void mqttReconnect() {
  Serial.println(F("----------------\r\nReconnecting to MQTT Broker..."));
  if (mqttClient.connect("LandSlidEtect_Gateway", GATEWAY_ACCESS_TOKEN, "")) {
    Serial.println(F("Reonnected to MQTT Broker"));
  } else {
    Serial.print(F("Failed with state "));
    Serial.print(mqttClient.state());
    Serial.println(F(" try again in 5 seconds"));
  }
}