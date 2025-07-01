#include <WiFi.h>
#include <PubSubClient.h>

// definisi pin
#define LDR_PIN 34
#define RELAY_PIN 16

// wifi
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""
/* #define WIFI_SSID "Redmi Note 13 Pro 5G"
#define WIFI_PASSWORD "12121212" */

// MQTT Broker
// #define MQTT_BROKER "202.0.107.154"
#define MQTT_BROKER "185.199.111.153"
// #define MQTT_BROKER "thingsboard.aiot.my.id"
#define MQTT_CLIENT_ID "gk1knjlm2q7ziovicfyl"
#define MQTT_USERNAME "bz3s6e4s6984x216el78"
#define MQTT_PASSWORD "50smc2bh7g6zmx9biswz"
#define MQTT_PORT 1983
#define MQTT_TOPIC_TELE_PUB "v1/devices/me/telemetry" // untuk publish
#define MQTT_TOPIC_RPC_REQ_SUB "v1/devices/me/rpc/request/+" // untuk RPC (remote procedural call) menerima data/perintah dari server
#define MQTT_TOPIC_BASE_RPC_RESP_PUB "v1/devices/me/rpc/response/" // untuk publish response dari RPC
#define MQTT_TOPIC_ATTR_SUB "v1/devices/me/attributes" // untuk subscribe atribut dari server
#define MQTT_TOPIC_BASE_ATTR_REPL_REQ_PUB "v1/devices/me/attributes/request/" // untuk publish request atribut ke server
#define MQTT_TOPIC_ATTR_REPL_SUB "v1/devices/me/attributes/response/+" // untuk subscribe atribut dari server cara lain

WiFiClient espClient;
PubSubClient client(espClient);

bool lampuState = false, sensorState = true, publishCahayaState = false, publishCahayaState2 = true;
String message, topicStr, responseRpc, publishMessage;
uint16_t telemetryPeriod = 1000;

void callback(char* topic, byte* payload, uint32_t length) {
  message = "";
  topicStr = String(topic);

  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.println("--------------------------------");
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(message);

  if (topicStr.indexOf("rpc") != -1) {
    int requestIdPos = topicStr.indexOf("v1/devices/me/rpc/request/") + 26;
    String requestIdStr = topicStr.substring(requestIdPos);
    String MQTT_TOPIC_RPC_RESP_PUB = MQTT_TOPIC_BASE_RPC_RESP_PUB + String(requestIdStr);

    if (message.indexOf("setLampuState") != -1) {
      if (message.substring(message.indexOf("params") + 8) == "true}") {
        responseRpc = "true";
        lampuState = true;
      } else {
        responseRpc = "false";
        lampuState = false;
      }

      sensorState = false; // kalau hidup atau matikan dari switch lampu di dashboard, sensor mati

      publishMessage = "{\"sensor_state\":false}";
      client.publish(MQTT_TOPIC_TELE_PUB, &publishMessage[0]);

      Serial.print("Publish: ");
      Serial.println(publishMessage);
    } else if (message.indexOf("setSensorState") != -1) {
      if (message.substring(message.indexOf("params") + 8) == "true}") {
        responseRpc = "true";
        sensorState = true;
      } else {
        responseRpc = "false";
        sensorState = false;
      }

      publishCahayaState = true;
      publishCahayaState2 = true;
    }

    Serial.print("Response: ");
    Serial.println(responseRpc);
    client.publish(&MQTT_TOPIC_RPC_RESP_PUB[0], &responseRpc[0]);
  } else if (topicStr.indexOf("attributes") != -1) {
    int attributePos = message.indexOf("tele_period");

    if (attributePos != -1) {
      attributePos += 13;
      
      if (message.substring(attributePos).toInt() >= 1000) {
        telemetryPeriod = message.substring(attributePos).toInt();
        Serial.print("Telemetry Publish Period: ");
        Serial.print(telemetryPeriod / 1000.);
        Serial.println("s");
      } else {
        telemetryPeriod = 1000;
        Serial.println("Telemetry period is too low than 1s, set telemetry period to 1s");
      }
    }
  }

  Serial.println("--------------------------------");
}

uint32_t requestAttributeId = 1;

void reconnect() {
  while (!client.connected()) {
    Serial.print("The client ");
    Serial.print(MQTT_CLIENT_ID);
    Serial.println(" connecting to the public MQTT broker");
    if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("Public EMQX MQTT broker connected");
    } else {
      Serial.print("Failed with state ");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }

  String MQTT_TOPIC_ATTR_REPL_REQ_PUB = MQTT_TOPIC_BASE_ATTR_REPL_REQ_PUB + String(requestAttributeId);
  client.publish(MQTT_TOPIC_ATTR_REPL_REQ_PUB.c_str(), "{\"clientKeys\":\"cliattr1,cliattr2\",\"sharedKeys\":\"tele_period,sharedattr2\"}");

  Serial.print("Topic: ");
  Serial.println(MQTT_TOPIC_ATTR_REPL_REQ_PUB);
  Serial.println("Publish: {\"clientKeys\":\"cliattr1,cliattr2\",\"sharedKeys\":\"tele_period,sharedattr2\"}");
  
  requestAttributeId++;

  publishMessage = "{\"lampu_state\":false}";
  client.publish(MQTT_TOPIC_TELE_PUB, &publishMessage[0]);

  Serial.print("Publish: ");
  Serial.println(publishMessage);

  publishMessage = "{\"sensor_state\":true}";
  client.publish(MQTT_TOPIC_TELE_PUB, &publishMessage[0]);

  Serial.print("Publish: ");
  Serial.println(publishMessage);

  client.subscribe(MQTT_TOPIC_RPC_REQ_SUB); // rpc subscribe
  client.subscribe(MQTT_TOPIC_ATTR_SUB); // telemetry period subscribe
  client.subscribe(MQTT_TOPIC_ATTR_REPL_SUB); // attribute request -> response subscribe
}

void setup() {
  Serial.begin(115200); // setel baud

  analogSetAttenuation(ADC_11db); // untuk membaca hingga 3.3V
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // agar relay mati pada awalnya

  // Connecting to a WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to the Wi-Fi network");

  // connecting to a mqtt broker
  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.print("The client ");
    Serial.print(MQTT_CLIENT_ID);
    Serial.println(" connecting to the public MQTT broker");
    if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("Public EMQX MQTT broker connected");
    } else {
      Serial.print("Failed with state ");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }

  String MQTT_TOPIC_ATTR_REPL_REQ_PUB = MQTT_TOPIC_BASE_ATTR_REPL_REQ_PUB + String(requestAttributeId);
  client.publish(MQTT_TOPIC_ATTR_REPL_REQ_PUB.c_str(), "{\"clientKeys\":\"cliattr1,cliattr2\",\"sharedKeys\":\"tele_period,sharedattr2\"}");

  Serial.print("Topic: ");
  Serial.println(MQTT_TOPIC_ATTR_REPL_REQ_PUB);
  Serial.println("Publish: {\"clientKeys\":\"cliattr1,cliattr2\",\"sharedKeys\":\"tele_period,sharedattr2\"}");
  
  requestAttributeId++;

  publishMessage = "{\"lampu_state\":false}";
  client.publish(MQTT_TOPIC_TELE_PUB, &publishMessage[0]);

  Serial.print("Publish: ");
  Serial.println(publishMessage);

  publishMessage = "{\"sensor_state\":true}";
  client.publish(MQTT_TOPIC_TELE_PUB, &publishMessage[0]);

  Serial.print("Publish: ");
  Serial.println(publishMessage);

  client.subscribe(MQTT_TOPIC_RPC_REQ_SUB); // rpc subscribe
  client.subscribe(MQTT_TOPIC_ATTR_SUB); // telemetry period subscribe
  client.subscribe(MQTT_TOPIC_ATTR_REPL_SUB); // attribute request -> response subscribe
}

bool publishCahayaState3 = true;
const float GAMMA = 0.7;
const float RL10 = 50;
float voltage, resistance, lux;
uint16_t analogValue;
uint32_t currentTime, pollingTime = 0;

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  if (sensorState) {
    // dijalankan setiap waktu berdasarkan telemetry period
    currentTime = millis();
    if (currentTime - pollingTime >= telemetryPeriod) {
      pollingTime = currentTime;

      analogValue = analogRead(LDR_PIN);
      voltage = analogValue / 4095. * 3.3;
      resistance = 2000 * voltage / (3.3 - voltage);
      lux = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA)) * 0.1;

      if (!isfinite(lux)) {
        lux = 100000;
      }

      Serial.print(pollingTime / 1000.);
      Serial.print("s : ");
      Serial.print(analogValue);
      Serial.print(" | ");
      Serial.print(voltage);
      Serial.print("V | ");
      Serial.print(resistance);
      Serial.print("Ohm | ");
      Serial.print(lux);
      Serial.println("lx");

      publishMessage = "{\"brightness\":" + String(lux) + "}";
      client.publish(MQTT_TOPIC_TELE_PUB, &publishMessage[0]);

      Serial.print("Publish: ");
      Serial.println(publishMessage);
    }
  } else {
    if (lampuState) {
      lux = 0;
    }
  }

  if (lampuState) { // lampu hidup
    digitalWrite(RELAY_PIN, HIGH);

    if (lux > 50) { // sensor hidup, terang
      lampuState = false;
    }
  } else { // lampu mati
    if (sensorState) { // sensor hidup
      if (lux <= 50) { // gelap
        publishCahayaState = true;
        if (publishCahayaState && publishCahayaState2) {
          publishMessage = "{\"lampu_state\":true}";
          client.publish(MQTT_TOPIC_TELE_PUB, &publishMessage[0]);

          Serial.print("Publish: ");
          Serial.println(publishMessage);

          publishCahayaState2 = false;
        }

        lampuState = true;
      } else { // terang
        if (publishCahayaState || publishCahayaState3) {
          publishMessage = "{\"lampu_state\":false}";
          client.publish(MQTT_TOPIC_TELE_PUB, &publishMessage[0]);

          Serial.print("Publish: ");
          Serial.println(publishMessage);

          publishCahayaState = false;
          publishCahayaState2 = true;
          publishCahayaState3 = false;
        }

        lampuState = false;
        digitalWrite(RELAY_PIN, LOW);
      }
    } else { // sensor mati
      digitalWrite(RELAY_PIN, LOW);
    }
  }
}
/**
 * kalau saklar lampu dihidup atau dimatikan, sensor cahayanya mati
 * sensor hidup
 *   kalau terang, lampu mati
 *     switch lampu di dashboard mati
 *   kalau gelap, lampu hidup
 *     switch lampu di dashboard hidup
 *   switch lampu dihidupkan dan switch sensor hidup
 *     lampu hidup
 *     sensor mati
 * lampu hidup karena switch lampu hidup
 *   switch lampu di dashboard hidup
 *   sensor mati
 * lampu mati karena switch lampu mati
 *   switch lampu di dashboard mati
 *   sensor mati
 * sensor mati
 *   switch sensor di dashboard mati
 *   lampu mati
 */