from paho.mqtt.client import Client
import base64
import json

# Callback saat berhasil terkoneksi ke broker
def on_connect(client, userdata, flags, rc):
    print("Connected with result code", str(rc))
    client.subscribe("GWID/40D63CFFFE83132F/UP")

# Callback saat menerima pesan
def on_message(client, userdata, msg):
    print(f"Topic: {msg.topic}")
    try:
        payload = json.loads(msg.payload.decode())
        b64_data = payload.get("data")
        if b64_data:
            decoded = base64.b64decode(b64_data).decode('utf-8')
            print("Decoded data:", decoded)
            # TODO: Forward ke ThingsBoard, Node-RED, Home Assistant
        else:
            print("Field 'data' tidak ditemukan")
    except Exception as e:
        print("Gagal memproses payload:", e)

# Inisialisasi klien MQTT dengan versi API yang baru
client = Client()
client.on_connect = on_connect
client.on_message = on_message

# Hubungkan ke broker MQTT
client.connect("broker.emqx.io", 1883, 60)

# Looping terus sampai dihentikan dengan Ctrl+C
try:
    client.loop_forever()
except KeyboardInterrupt:
    print("\nProgram dihentikan dengan Ctrl+C")
    client.disconnect()