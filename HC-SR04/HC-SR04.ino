#include <WiFi.h>
#include <PubSubClient.h>

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
// #define CM_TO_INCH 0.393701

#define TRIG_PIN 12
#define ECHO_PIN 14
#define LED_PIN 23
#define LED_PIN_ESP 2
#define BUZZER_PIN 19

// WiFi
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""
/* #define WIFI_SSID "Redmi Note 13 Pro 5G"
#define WIFI_PASSWORD "12121212" */

// MQTT Broker
#define MQTT_BROKER "202.0.107.154"
#define MQTT_CLIENT_ID "ngbwkezpf0jy8k5jql6l"
#define MQTT_USERNAME "zsxetv8eaqrewlq5ztaj"
#define MQTT_PASSWORD "b1kjtw5mabmba3sjmwgx"
#define MQTT_PORT 1983
#define MQTT_TOPIC_TELE_PUB "v1/devices/me/telemetry" // untuk publish
#define MQTT_TOPIC_RPC_REQ_SUB "v1/devices/me/rpc/request/+" // untuk RPC (remote procedural call) menerima data/perintah dari server
String MQTT_TOPIC_BASE_RPC_RESP_PUB = "v1/devices/me/rpc/response/"; // untuk publish response dari RPC
#define MQTT_TOPIC_ATTR_SUB "v1/devices/me/attributes" // untuk subscribe atribut dari server
#define MQTT_TOPIC_BASE_ATTR_REPL_REQ_PUB "v1/devices/me/attributes/request/" // untuk publish request atribut ke server
#define MQTT_TOPIC_ATTR_REPL_SUB "v1/devices/me/attributes/response/+" // untuk subscribe atribut dari server cara lain

// Variable declaration
uint32_t currentTime, pollingTime = 0, requestAttributeId = 1;
unsigned long duration;
float distanceCm = 100;
// float distanceInch;
bool wailing = false, rpcState = false, innerLedState = false;
unsigned int frequency = 600, lastToneChange, currentTime2, telemetryPeriod = 2000, teleTime = 0;
int8_t direction = 1;
String publishMessage = "";

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  String topicStr = String(topic);

  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
    // Serial.print((char)payload[i]);
  }

  Serial.println("--------------------------------");
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(message);

  if(topicStr.indexOf("rpc") != -1){
    String responseRpc = "{\"";

    if (message.indexOf("setGpioStatus") >= 0) {
      if (message.substring(message.indexOf("enabled") + 9) == "true}}") {
        responseRpc += message.substring((message.indexOf("pin") + 5), (message.indexOf("pin") + 6)) + "\":";
        responseRpc += "true}";
        rpcState = true;
      } else {
        responseRpc += message.substring((message.indexOf("pin") + 5), (message.indexOf("pin") + 6)) + "\":";
        responseRpc += "false}";
        rpcState = false;
      }
    } else {
      if (message.substring(message.indexOf("params") + 8) == "true}") {
        responseRpc += "params\":true}";
        innerLedState = true;
      } else {
        responseRpc += "params\":false}";
        innerLedState = false;
      }
    }

    int requestIdPos = topicStr.indexOf("v1/devices/me/rpc/request/") + 26;
    String requestIdStr = topicStr.substring(requestIdPos);
    String MQTT_TOPIC_RPC_RESP_PUB = MQTT_TOPIC_BASE_RPC_RESP_PUB + requestIdStr;

    // Serial.println(requestIdPos);
    // Serial.println(requestIdStr);
    // Serial.println(MQTT_TOPIC_RPC_RESP_PUB);

    Serial.println(responseRpc);
    client.publish(&MQTT_TOPIC_RPC_RESP_PUB[0], &responseRpc[0]);
  }else if(topicStr.indexOf("attributes") != -1){
    int attributePos = message.indexOf("tele_period");

    if(attributePos != -1){
      attributePos += 13;

      // int endingChar = message.substring("}", attributePos);
      telemetryPeriod = message.substring(attributePos).toInt();
      Serial.print("Telemetry Publish Period : ");
      Serial.println(telemetryPeriod);
    }
  }

  Serial.println("--------------------------------");
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect to the MQTT broker
    if (client.connect(MQTT_CLIENT_ID)) {
      Serial.println("connected");
      // Subscribe to the temperature topic
      client.subscribe(MQTT_TOPIC_RPC_REQ_SUB);
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

/* void playWailingTone() {
  // ambulans di jalan untuk membuka jalan
  for (int i = 600; i <= 1200; i += 5) {
    tone(BUZZER_PIN, i);
    delay(20);
  }
  for (int i = 1200; i >= 600; i -= 5) {
    tone(BUZZER_PIN, i);
    delay(20);
  }

  // sirene bahaya nuklir
  for (int i = 300; i <= 1200; i += 2) {
    tone(BUZZER_PIN, i);
    delay(20);
  }
  for (int i = 1200; i >= 300; i -= 5) {
    tone(BUZZER_PIN, i);
    delay(20);
  }
} */

void playWailingTone() {
  if (!wailing) {
    wailing = true;
    tone(BUZZER_PIN, frequency);
  }

  currentTime2 = millis();
  if (currentTime2 - lastToneChange >= 20) {
    lastToneChange = currentTime2;
    frequency += direction * 5;
    if (frequency >= 1200 || frequency <= 600) {
      direction *= -1;
    }
    tone(BUZZER_PIN, frequency);
  }
}

void setup() {
  // Set software serial baud to 115200;
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT); // Sets the TRIG_PIN as an Output
  pinMode(ECHO_PIN, INPUT); // Sets the ECHO_PIN as an Input

  // Connecting to a WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to the Wi-Fi network");

  // connecting to a mqtt broker
  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setCallback(callback);

  while (!client.connected()) {
    // client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public MQTT broker\n", MQTT_CLIENT_ID);
    if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("Public EMQX MQTT broker connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }

  String MQTT_TOPIC_ATTR_REPL_REQ_PUB = MQTT_TOPIC_BASE_ATTR_REPL_REQ_PUB + String(requestAttributeId);
  client.publish(MQTT_TOPIC_ATTR_REPL_REQ_PUB.c_str(),"{\"clientKeys\":\"cliattr1,cliattr2\",\"sharedKeys\":\"tele_period,sharedattr2\"}");
  requestAttributeId++;

  client.subscribe(MQTT_TOPIC_RPC_REQ_SUB); // rpc subscribe
  client.subscribe(MQTT_TOPIC_ATTR_SUB); // telemetry period subscribe
  client.subscribe(MQTT_TOPIC_ATTR_REPL_SUB); // attribute request -> response subscribe

  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_ESP, OUTPUT);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  currentTime = millis();
  if(currentTime - pollingTime >= 1000){
    pollingTime = currentTime;

    // Clears the TRIG_PIN
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    // Sets the TRIG_PIN on HIGH state for 10 micro seconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
    duration = pulseIn(ECHO_PIN, HIGH);
    
    // Calculate the distance
    distanceCm = duration * SOUND_SPEED / 2;
    // distanceCm = duration / 58;
    
    // Convert to inches
    // distanceInch = distanceCm * CM_TO_INCH;

    /* if (distanceCm <= 30) {
      digitalWrite(LED_PIN, HIGH);
      playWailingTone();
    } else {
      digitalWrite(LED_PIN, LOW);
      noTone(BUZZER_PIN);
    } */
    
    // Prints the distance in the Serial Monitor
    /* Serial.print("Distance (cm): ");
    Serial.println(distanceCm);
    
    Serial.print("Distance (inch): ");
    Serial.println(distanceInch); */

    Serial.print(pollingTime / 1000);
    Serial.print("s : ");
    Serial.print(distanceCm);
    Serial.println("cm");

    if(currentTime - teleTime >= telemetryPeriod){
      teleTime = currentTime;

      publishMessage = "{\"jarak\":" + String(distanceCm) + "}";
      client.publish(MQTT_TOPIC_TELE_PUB, &publishMessage[0]);

      Serial.print(teleTime / 1000);
      Serial.print("s : ");
      Serial.print("Publish ");
      Serial.println(publishMessage);
    }
  }

  if (distanceCm <= 30 || rpcState) {
    digitalWrite(LED_PIN, HIGH);
    playWailingTone();
  } else {
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
    frequency = 600;
    direction = 1;
    wailing = false;
  }

  if (innerLedState) {
    digitalWrite(LED_PIN_ESP, HIGH);
  } else {
    digitalWrite(LED_PIN_ESP, LOW);
  }
}