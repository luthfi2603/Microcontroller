#include <HTTPClient.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "secrets.h"

// Deklarasi variabel global
constexpr const char* MQTT_TOPIC_TELE_PUB = "v1/devices/me/telemetry";
constexpr const char* MQTT_TOPIC_GATEWAY_TELE_PUB = "v1/gateway/telemetry";
WiFiClient wiFiClient;
WiFiClientSecure secureWiFiClient;
PubSubClient mqttClient(wiFiClient);

// Prototipe fungsi
String urlEncode(const String &str);
void sendMessageToTelegram(const String &message);
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

  sendMessageToTelegram(urlEncode("Halo dari ESP32!\nフリーナちゃんとビビアンくん"));
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

    const char *gatewayPayload = "{\"ax\":0.5}";
    const char *payload = "{\"Node 1\":[{\"ax\":0.6}],\"Node 2\":[{\"ax\":0.7}]}";
    Serial.println(F("----------------\r\nPublish:"));
    Serial.println(gatewayPayload);
    Serial.println(payload);
    mqttClient.publish(MQTT_TOPIC_TELE_PUB, gatewayPayload);
    mqttClient.publish(MQTT_TOPIC_GATEWAY_TELE_PUB, payload);
  }
}

String urlEncode(const String &str) {
  String encodedString = "";

  for (int i = 0; i < str.length(); i++) {
    uint8_t c = str.charAt(i);

    // Menurut RFC 3986, huruf, angka, dan 4 simbol ini tidak boleh di-encode
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      encodedString += (char)c;
    } else { // Sisanya (termasuk spasi dan simbol unik), ubah ke format %HEX
      char hex[4];
      sprintf(hex, "%%%02X", c);
      encodedString += hex;
    }
  }

  return encodedString;
}

void sendMessageToTelegram(const String &message) {
  HTTPClient http;

  String apiUrl = String("https://api.telegram.org/bot") + TELEGRAM_BOT_API_TOKEN + "/sendMessage?chat_id=" + CHAT_ID + "&text=" + message;
  
  Serial.println(F("----------------\r\nTrying request to: "));
  Serial.println(apiUrl);

  http.begin(secureWiFiClient, apiUrl);

  int16_t httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    Serial.print(F("HTTP Response Code: "));
    Serial.println(httpResponseCode);
    
    String payload = http.getString();
    Serial.println(F("Server Response:"));
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