from paho.mqtt.client import Client, CallbackAPIVersion
import base64
import json
import re

# Callback saat berhasil terkoneksi ke broker
def on_connect(client, userdata, flags, reason_code, properties=None):
    print("Connected with result code:", reason_code)
    print("----------------")
    client.subscribe("GWID/40D63CFFFE83132F/UP")

# Memisahkan antara json di string payload
def split_json_objects(raw):
    # Cari semua potongan JSON dalam satu string payload
    return re.findall(r'\{.*?\}', raw)

# Callback saat menerima pesan
def on_message(client, userdata, msg):
    print(f"Topic: {msg.topic}")
    data_str = msg.payload.decode()

    try:
        json_parts = split_json_objects(data_str)

        for part in json_parts:
            try:
                payload = json.loads(part)
                b64_data = payload.get("data")

                if b64_data:
                    try:
                        decoded = base64.b64decode(b64_data).decode('utf-8')
                        print("################\nDecoded data:")
                        print(decoded)
                        # TODO kirimkan ke ThingsBoard, Node-Red, Home Assistant
                    except Exception as decode_error:
                        print("Gagal decode base64:", decode_error)
                else:
                    print("Field 'data' tidak ditemukan")
            except Exception as json_error:
                print("Gagal parsing JSON:", json_error)
    except Exception as e:
        print("Gagal memproses payload:", e)
    print("----------------")

# Inisialisasi klien MQTT dengan versi API 2
client = Client(callback_api_version=CallbackAPIVersion.VERSION2)
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