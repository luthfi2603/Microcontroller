#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "secrets.h"

// Deklarasi variabel
uint32_t currentTime, pollingTime = 0;

// Prototipe fungsi
String urlEncode(const String &str);
void sendMessageToTelegram(const String &message);

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Menghubungkan ke jaringan Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print(F("\r\nConnecting to Wi-Fi"));
  while (WiFi.status() != WL_CONNECTED) {
    currentTime = millis();
    if(currentTime - pollingTime >= 490){
      pollingTime = currentTime;

      Serial.print(F("."));

      delay(10);
    }
  }
  Serial.println(F("\r\nConnected to the Wi-Fi network"));
  Serial.println(F("------------------------------"));

  sendMessageToTelegram(urlEncode("Halo dari ESP32!\nフリーナちゃんとビビアンくん"));
}

void loop() {

}

/**
 * Deklarasi variabel tipe data primitif lebih bagus sedekat mungkin ke tempat dipakainya
 * atau scope-nya jadi bagus di dalam loop
 * Tapi kalau objek sebaiknya di luar loop seperti String itu
 */
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

/**
 * Variabel tipe data primitif dan objek stack seperti httpResponseCode, apiUrl, HTTPClient http
 * seluruh memori Stack yang dipakai oleh variabel-variabel ini langsung
 * dibebaskan saat itu juga tanpa sisa ketika program berjalan setelah tutup } (setelah keluar
 * dari scope) dari if-statement
 * 
 * Variabel heap WiFiClientSecure *client = new WiFiClientSecure
 * karena deklarasi pakai new tidak akan dibersihkan otomatis setelah tutup } fungsinya
 * tapi karena melakukan delete maka memori dari variabel itu akan dibersihkan
 */
void sendMessageToTelegram(const String &message) {
  /**
   * client itu stack
   * new WiFiClientSecure itu heap
   * jadi kita harus menghancurkan new WiFiClientSecure setelah selesai dipakai
   * dengan delete client
   */
  WiFiClientSecure *client = new WiFiClientSecure; // Pesan pointer di SRAM
  
  if (client) { // Cek apakah berhasil (tidak NULL)
    client->setInsecure(); // Bypass verifikasi sertifikat SSL

    HTTPClient http;

    // https://jsonplaceholder.typicode.com/todos/1
    String apiUrl = String("https://api.telegram.org/bot") + TELEGRAM_BOT_API_TOKEN + "/sendMessage?chat_id=" + CHAT_ID + "&text=" + message;
    
    Serial.print(F("Trying request to: "));
    Serial.println(apiUrl);

    http.begin(*client, apiUrl);

    uint8_t httpResponseCode = http.GET();

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
    delete client;
    client = nullptr;
    Serial.println(F("------------------------------"));
  } else {
    Serial.println(F("Failed to make object WiFiClientSecure!"));
  }
}