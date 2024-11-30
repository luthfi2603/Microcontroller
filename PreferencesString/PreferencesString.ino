#include <Preferences.h>

Preferences preferences; // namespace dan key max cuma 15 character

void setup() {
  Serial.begin(115200);
  
  preferences.begin("test_string", false);
  String string = "ini adalah string";
  preferences.putString("ini_string", string.c_str());
  preferences.end();

  preferences.begin("test_string", false);
  Serial.println();
  Serial.print("hasil: ");
  Serial.println(preferences.getString("ini_string", "kok gak ada"));
  
  Serial.println("restarting in 10 seconds...");
  delay(10000);
  ESP.restart();
}

void loop() {

}