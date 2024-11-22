#include <painlessMesh.h>
#include <AsyncTCP.h>

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
// #define CM_TO_INCH 0.393701

#define TRIG_PIN 12
#define ECHO_PIN 14

#define MESH_PREFIX "MeshNetwork"
#define MESH_PASSWORD "12345678"
#define MESH_PORT 5555

Scheduler userScheduler;
painlessMesh mesh;

uint64_t duration;
float distanceCm;

// Task untuk mengirim data DHT
Task sendTask(10000, TASK_FOREVER, []() {
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

    String msg = "{\"distance\":" + String(distanceCm) + "}";
    mesh.sendBroadcast(msg);
    Serial.println("Data Sent: " + msg);
});

void setup() {
    Serial.begin(115200);
    pinMode(TRIG_PIN, OUTPUT); // Sets the TRIG_PIN as an Output
    pinMode(ECHO_PIN, INPUT); // Sets the ECHO_PIN as an Input
    
    mesh.setDebugMsgTypes(ERROR | STARTUP);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    
    // Callback untuk menerima pesan
    mesh.onReceive([](uint32_t from, String &msg) {
        Serial.printf("Received from %u: %s\n", from, msg.c_str());
    });

    // Tambahkan task ke scheduler dan aktifkan
    userScheduler.addTask(sendTask);
    sendTask.enable();
}

void loop() {
    mesh.update();
}