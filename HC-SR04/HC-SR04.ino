#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
// #define CM_TO_INCH 0.393701

#define TRIG_PIN 12
#define ECHO_PIN 14
#define LED_PIN 23
#define LED_PIN_ESP 2
#define BUZZER_PIN 19
#define BOOT_BUTTON_PIN 0

// WiFi
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""
/* #define WIFI_SSID "Redmi Note 13 Pro 5G"
#define WIFI_PASSWORD "12121212" */

// MQTT Broker
// #define MQTT_BROKER "202.0.107.154"
#define MQTT_BROKER "thingsboard.aiot.my.id"
#define MQTT_CLIENT_ID "ngbwkezpf0jy8k5jql6l"
#define MQTT_USERNAME "zsxetv8eaqrewlq5ztaj"
#define MQTT_PASSWORD "b1kjtw5mabmba3sjmwgx"
#define MQTT_PORT 1983
#define MQTT_TOPIC_TELE_PUB "v1/devices/me/telemetry" // untuk publish
#define MQTT_TOPIC_RPC_REQ_SUB "v1/devices/me/rpc/request/+" // untuk RPC (remote procedural call) menerima data/perintah dari server
#define MQTT_TOPIC_BASE_RPC_RESP_PUB "v1/devices/me/rpc/response/" // untuk publish response dari RPC
#define MQTT_TOPIC_ATTR_SUB "v1/devices/me/attributes" // untuk subscribe atribut dari server
#define MQTT_TOPIC_BASE_ATTR_REPL_REQ_PUB "v1/devices/me/attributes/request/" // untuk publish request atribut ke server
#define MQTT_TOPIC_ATTR_REPL_SUB "v1/devices/me/attributes/response/+" // untuk subscribe atribut dari server cara lain

// Preferences key
#define AP_NAME "ap_name"
#define AP_PASSWORD "ap_password"

// Variable declaration
uint32_t currentTime, pollingTime = 0, requestAttributeId = 1, lastDebounceTime = 0;
unsigned long duration;
float distanceCm = 100;
// float distanceInch;
bool wailing = false, rpcState = false, innerLedState = false, reconnectState = true;
unsigned int frequency = 600, lastToneChange, currentTime2, telemetryPeriod = 2000, teleTime = 0;
int8_t direction = 1, reading, lastButtonState = HIGH, buttonState;
String publishMessage = "", newWifiAccessPoint = "", newWifiPassword = "", responseRpc;
unsigned char accessPointAttribute = 0;

WiFiClient espClient;
PubSubClient client(espClient);
Preferences database;

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
    if (message.indexOf("setGpioStatus") >= 0) {
      if (message.substring(message.indexOf("enabled") + 9) == "true}}") {
        responseRpc = "{\"";
        responseRpc += message.substring((message.indexOf("pin") + 5), (message.indexOf("pin") + 6)) + "\":";
        responseRpc += "true}";
        rpcState = true;

        // clear preferences
        if (message.substring((message.indexOf("pin") + 5), (message.indexOf("pin") + 6)) == "1") {
          database.begin("esp32db", false);
          database.clear();
          database.end();

          Serial.println("Semua preferences dihapus");
        }
      } else {
        responseRpc = "{\"";
        responseRpc += message.substring((message.indexOf("pin") + 5), (message.indexOf("pin") + 6)) + "\":";
        responseRpc += "false}";
        rpcState = false;
      }
    } else if (message.indexOf("getGpioStatus") != -1) {
      responseRpc = "{\"1\":false,\"2\":false,\"3\":false}";
    } else if (message.indexOf("getValue") != -1) {
      if (innerLedState) {
        responseRpc = "true";
      } else {
        responseRpc = "false";
      }
    } else if (message.indexOf("setValue") != -1) {
      if (message.substring(message.indexOf("params") + 8) == "true}") {
        responseRpc = "true";
        innerLedState = true;
      } else {
        responseRpc = "false";
        innerLedState = false;
      }

      database.begin("esp32db", false);
      Serial.print("ap_name:");
      Serial.println(database.getString(AP_NAME, ""));
      Serial.print("ap_password:");
      Serial.println(database.getString(AP_PASSWORD, ""));
      database.end();
    } 

    int requestIdPos = topicStr.indexOf("v1/devices/me/rpc/request/") + 26;
    String requestIdStr = topicStr.substring(requestIdPos);
    String MQTT_TOPIC_RPC_RESP_PUB = MQTT_TOPIC_BASE_RPC_RESP_PUB + requestIdStr;

    // Serial.println(requestIdPos);
    // Serial.println(requestIdStr);
    // Serial.println(MQTT_TOPIC_RPC_RESP_PUB);

    Serial.print("Response: ");
    Serial.println(responseRpc);
    client.publish(&MQTT_TOPIC_RPC_RESP_PUB[0], &responseRpc[0]);
  }else if(topicStr.indexOf("attributes") != -1){
    // message : {"...:"..},{"tele_period":2000},{"wifi_access_point":"aptes"},{"wifi_password":"aptes"}
    int attributePos = message.indexOf("tele_period");
    int wifiAccessPointPosition = message.indexOf("wifi_access_point");
    int wifiPasswordPosition = message.indexOf("wifi_password");

    if (attributePos != -1) {
      attributePos += 13;

      // int endingChar = message.substring("}", attributePos);
      if (message.substring(attributePos).toInt() >= 1000) {
        telemetryPeriod = message.substring(attributePos).toInt();
        Serial.print("Telemetry Publish Period : ");
        Serial.println(telemetryPeriod);
      } else {
        telemetryPeriod = 1000;
        Serial.println("Telemetry period is too low than 1s, set telemetry period to 1s");
      }
    } else if (wifiAccessPointPosition != -1) {
      wifiAccessPointPosition += 20;
      int endingChar = message.indexOf("\"}", wifiAccessPointPosition);
      newWifiAccessPoint = message.substring(wifiAccessPointPosition, endingChar);
      Serial.println(newWifiAccessPoint);

      database.begin("esp32db", false);
      database.putString(AP_NAME, newWifiAccessPoint.c_str());

      Serial.print("ap_name:");
      Serial.println(database.getString(AP_NAME, ""));

      database.end();

      accessPointAttribute |= 1;   // x x x x x x x 1
    } else if (wifiPasswordPosition != -1) {
      wifiPasswordPosition += 16;
      int endingChar = message.indexOf("\"}", wifiPasswordPosition);
      newWifiPassword = message.substring(wifiPasswordPosition, endingChar);
      Serial.println(newWifiPassword);

      database.begin("esp32db", false);
      database.putString(AP_PASSWORD, newWifiPassword.c_str());

      Serial.print("ap_password:");
      Serial.println(database.getString(AP_PASSWORD, ""));

      database.end();

      accessPointAttribute |= 2; // x x x x x x 1 x
    }

    if (accessPointAttribute == 3) {
      WiFi.disconnect();
      WiFi.begin(newWifiAccessPoint, newWifiPassword);
      accessPointAttribute = 0;

      Serial.print("Connecting to Wi-Fi");
      while (WiFi.status() != WL_CONNECTED) {
        currentTime = millis();
        if(currentTime - pollingTime >= 500){
          pollingTime = currentTime;

          Serial.print(".");
        }

        // clear preferences
        reading = digitalRead(BOOT_BUTTON_PIN);

        // Jika status button berubah (karena noise atau tekanan), reset timer debounce
        if (reading != lastButtonState) {
          lastDebounceTime = millis();
        }

        // Cek apakah sudah melewati waktu debounce
        if ((millis() - lastDebounceTime) > 50) {
          // Jika status button sudah stabil
          if (reading != buttonState) {
            buttonState = reading;

            // Hanya ubah status LED jika button ditekan (LOW)
            if (buttonState == LOW) {
              database.begin("esp32db", false);
              database.clear();
              database.end();

              Serial.println();
              Serial.println("Semua preferences dihapus");
              ESP.restart();
            }
          }
        }

        // Simpan status button terakhir
        lastButtonState = reading;
      }
      Serial.println();
      Serial.println("Connected to the Wi-Fi network");
    }
  }

  Serial.println("--------------------------------");
}

void reconnect() {
  while (!client.connected()) {
    if (!reconnectState) {
      Serial.print("The client ");
      Serial.print(MQTT_CLIENT_ID);
      Serial.println(" connecting to the public MQTT broker");
      if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
        Serial.println("Public EMQX MQTT broker connected");
      } else {
        Serial.print("Failed with state ");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
      }
      reconnectState = true;
    } else {
      currentTime = millis();
      if(currentTime - pollingTime >= 5000){
        pollingTime = currentTime;

        Serial.print("The client ");
        Serial.print(MQTT_CLIENT_ID);
        Serial.println(" connecting to the public MQTT broker");
        if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
          Serial.println("Public EMQX MQTT broker connected");
        } else {
          Serial.print("Failed with state ");
          Serial.print(client.state());
          Serial.println(" try again in 5 seconds");
        }
      }
    }

    // clear preferences
    reading = digitalRead(BOOT_BUTTON_PIN);

    // Jika status button berubah (karena noise atau tekanan), reset timer debounce
    if (reading != lastButtonState) {
      lastDebounceTime = millis();
    }

    // Cek apakah sudah melewati waktu debounce
    if ((millis() - lastDebounceTime) > 50) {
      // Jika status button sudah stabil
      if (reading != buttonState) {
        buttonState = reading;

        // Hanya ubah status LED jika button ditekan (LOW)
        if (buttonState == LOW) {
          database.begin("esp32db", false);
          database.clear();
          database.end();

          Serial.println("Semua preferences dihapus");
          ESP.restart();
        }
      }
    }

    // Simpan status button terakhir
    lastButtonState = reading;
  }

  String MQTT_TOPIC_ATTR_REPL_REQ_PUB = MQTT_TOPIC_BASE_ATTR_REPL_REQ_PUB + String(requestAttributeId);
  client.publish(MQTT_TOPIC_ATTR_REPL_REQ_PUB.c_str(), "{\"clientKeys\":\"cliattr1,cliattr2\",\"sharedKeys\":\"tele_period,sharedattr2\"}");
  requestAttributeId++;

  client.subscribe(MQTT_TOPIC_RPC_REQ_SUB); // rpc subscribe
  client.subscribe(MQTT_TOPIC_ATTR_SUB); // telemetry period subscribe
  client.subscribe(MQTT_TOPIC_ATTR_REPL_SUB); // attribute request -> response subscribe
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
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_ESP, OUTPUT);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  /* tombol bawakan esp32, INPUT_PULLUP 
  agar memastikan tombol bisa HIGH dan LOW dengan menghubungkannya ke VCC, dan ketika ditekan
  tombol akan dihubungkan ke GND sehingga dia jadi LOW */
  
  database.begin("esp32db", false);

  // Connecting to a WiFi network
  newWifiAccessPoint = database.getString(AP_NAME, "");
  newWifiPassword = database.getString(AP_PASSWORD, "");
  database.end();

  if (newWifiAccessPoint != "" && newWifiPassword != "") {
    WiFi.begin(newWifiAccessPoint, newWifiPassword);
    Serial.println("Menggunakan data dari preferences");
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    currentTime = millis();
    if(currentTime - pollingTime >= 500){
      pollingTime = currentTime;

      Serial.print(".");
    }

    // clear preferences
    reading = digitalRead(BOOT_BUTTON_PIN);

    // Jika status button berubah (karena noise atau tekanan), reset timer debounce
    if (reading != lastButtonState) {
      lastDebounceTime = millis();
    }

    // Cek apakah sudah melewati waktu debounce
    if ((millis() - lastDebounceTime) > 50) {
      // Jika status button sudah stabil
      if (reading != buttonState) {
        buttonState = reading;

        // Hanya ubah status LED jika button ditekan (LOW)
        if (buttonState == LOW) {
          database.begin("esp32db", false);
          database.clear();
          database.end();

          Serial.println();
          Serial.println("Semua preferences dihapus");
          ESP.restart();
        }
      }
    }

    // Simpan status button terakhir
    lastButtonState = reading;
  }
  Serial.println();
  Serial.println("Connected to the Wi-Fi network");

  // connecting to a mqtt broker
  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setCallback(callback);

  while (!client.connected()) {
    if (reconnectState) {
      Serial.print("The client ");
      Serial.print(MQTT_CLIENT_ID);
      Serial.println(" connecting to the public MQTT broker");
      if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
        Serial.println("Public EMQX MQTT broker connected");
      } else {
        Serial.print("Failed with state ");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
      }
      reconnectState = false;
    } else {
      currentTime = millis();
      if(currentTime - pollingTime >= 5000){
        pollingTime = currentTime;

        Serial.print("The client ");
        Serial.print(MQTT_CLIENT_ID);
        Serial.println(" connecting to the public MQTT broker");
        if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
          Serial.println("Public EMQX MQTT broker connected");
        } else {
          Serial.print("Failed with state ");
          Serial.print(client.state());
          Serial.println(" try again in 5 seconds");
        }
      }
    }

    // clear preferences
    reading = digitalRead(BOOT_BUTTON_PIN);

    // Jika status button berubah (karena noise atau tekanan), reset timer debounce
    if (reading != lastButtonState) {
      lastDebounceTime = millis();
    }

    // Cek apakah sudah melewati waktu debounce
    if ((millis() - lastDebounceTime) > 50) {
      // Jika status button sudah stabil
      if (reading != buttonState) {
        buttonState = reading;

        // Hanya ubah status LED jika button ditekan (LOW)
        if (buttonState == LOW) {
          database.begin("esp32db", false);
          database.clear();
          database.end();

          Serial.println("Semua preferences dihapus");
          ESP.restart();
        }
      }
    }

    // Simpan status button terakhir
    lastButtonState = reading;
  }

  String MQTT_TOPIC_ATTR_REPL_REQ_PUB = MQTT_TOPIC_BASE_ATTR_REPL_REQ_PUB + String(requestAttributeId);
  client.publish(MQTT_TOPIC_ATTR_REPL_REQ_PUB.c_str(), "{\"clientKeys\":\"cliattr1,cliattr2\",\"sharedKeys\":\"tele_period,sharedattr2\"}");

  Serial.print("Topic: ");
  Serial.println(MQTT_TOPIC_ATTR_REPL_REQ_PUB);
  Serial.println("Publish: {\"clientKeys\":\"cliattr1,cliattr2\",\"sharedKeys\":\"tele_period,sharedattr2\"}");

  requestAttributeId++;

  client.subscribe(MQTT_TOPIC_RPC_REQ_SUB); // rpc subscribe
  client.subscribe(MQTT_TOPIC_ATTR_SUB); // telemetry period subscribe
  client.subscribe(MQTT_TOPIC_ATTR_REPL_SUB); // attribute request -> response subscribe
}

void loop() {
  if (!client.connected()) {
    reconnect();

    reconnectState = false;
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

  // clear preferences
  reading = digitalRead(BOOT_BUTTON_PIN);

  // Jika status button berubah (karena noise atau tekanan), reset timer debounce
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  // Cek apakah sudah melewati waktu debounce
  if ((millis() - lastDebounceTime) > 50) {
    // Jika status button sudah stabil
    if (reading != buttonState) {
      buttonState = reading;

      // Hanya ubah status LED jika button ditekan (LOW)
      if (buttonState == LOW) {
        database.begin("esp32db", false);
        database.clear();
        database.end();

        Serial.println("Semua preferences dihapus");
        ESP.restart();
      }
    }
  }

  // Simpan status button terakhir
  lastButtonState = reading;
}