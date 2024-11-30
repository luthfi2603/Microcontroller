#include <Preferences.h>

Preferences preferences;

void setup() {
  Serial.begin(115200);
  Serial.println();

  preferences.begin("esp32db", false);

  // Remove all preferences under the opened namespace
  preferences.clear();
  Serial.println("All preferences are cleared");

  // Or remove the counter key only
  // preferences.remove("key_name");
  // Serial.println("Key name is cleared");
}

void loop() {

}