#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "secrets.h"

// Deklarasi variabel
uint32_t currentTime, pollingTime = 0;

// Prototipe fungsi
String urlEncode(String str);

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Menghubungkan ke jaringan Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("\r\nConnecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    currentTime = millis();
    if(currentTime - pollingTime >= 490){
      pollingTime = currentTime;

      Serial.print(".");

      delay(10);
    }
  }
  Serial.println("\r\nConnected to the Wi-Fi network");
  Serial.println("------------------------------");

  WiFiClientSecure *client = new WiFiClientSecure; // Pesan pointer di RAM
  
  if (client) { // Cek apakah berhasil (tidak NULL)
    client->setInsecure(); // Bypass verifikasi sertifikat SSL

    HTTPClient http;

    String message = urlEncode("Halo dari ESP32!\nフリーナちゃんとビビアンくん");
    // https://jsonplaceholder.typicode.com/todos/1
    String apiUrl = String("https://api.telegram.org/bot") + TELEGRAM_BOT_API_TOKEN + "/sendMessage?chat_id=" + chatId + "&text=" + message;
    
    Serial.print("Trying request to: ");
    Serial.println(apiUrl);

    http.begin(*client, apiUrl);

    uint8_t httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      Serial.print("HTTP Response Code: ");
      Serial.println(httpResponseCode);
      
      String payload = http.getString();
      Serial.println("Server Response:");
      Serial.println(payload);
    } else {
      Serial.print("Error Code: ");
      Serial.println(httpResponseCode);
    }

    http.end();
    delete client;
    client = nullptr;
    Serial.println("------------------------------");
  } else {
    Serial.println("Failed to make object WiFiClientSecure!");
  }
}

void loop() {

}

/**
 * Deklarasi variabel tipe data primitif lebih bagus sedekat mungkin ke tempat dipakainya
 * atau scope-nya jadi bagus di dalam loop
 * Tapi kalau objek sebaiknya di luar loop seperti String itu
 */
String urlEncode(String str) {
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