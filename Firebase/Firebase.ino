#include <WiFi.h>
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

#define DATABASE_URL "https://piot-luthfi-default-rtdb.firebaseio.com/"
#define API_KEY "AIzaSyAvyOLNoB8Jto0s7yBh0kpw2JcDXcxqRhg"

#define ssid "Redmi Note 13 Pro 5G"
#define password "12121212"
#define TRIGGER_PIN 12
#define ECHO_PIN 14

FirebaseData data;
FirebaseAuth auth;
FirebaseConfig config;

void setup() {
  Serial.begin(115200);

  // Menghubungkan ke WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected to WiFi: ");
  Serial.println(WiFi.localIP());

  // Konfigurasi Firebase
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;
  Firebase.signUp(&config, &auth, "", "");

  // Inisialisasi Firebase
  Firebase.begin(&config, &auth);
  Firebase.reconnectNetwork(true);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

float measureDistance() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2; // Konversi waktu ke jarak dalam cm
  return distance;
}

void loop() {
  float distance = measureDistance();

  // Kirim data jarak ke Firebase
  if (Firebase.ready()) {
    bool successDistance = Firebase.setFloat(data, "/SensorUltrasonic/distance", distance);
    Serial.printf("Set float SensorUltrasonic/distance... %s\n", successDistance ? "ok" : data.errorReason().c_str());
  } else {
    Serial.println("Firebase not ready");
  }

  // Tampilkan data ke serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(1000); // Delay untuk pengukuran setiap 1 detik
}