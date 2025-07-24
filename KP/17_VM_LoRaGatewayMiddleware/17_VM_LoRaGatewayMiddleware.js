// Mengecek apakah payload ada
if (msg.payload) {
    try {
        // Fungsi untuk memisahkan objek JSON dalam satu string panjang (mirip split_json_objects)
        let json_parts = msg.payload.match(/\{.*?\}/g);

        if (!json_parts || json_parts.length === 0) {
            node.warn("Tidak ada objek JSON ditemukan di dalam payload.");
            msg.payload = { error: "Tidak ditemukan objek JSON dalam payload" };
            return msg;
        }

        // Array untuk menyimpan hasil decode
        let hasil = [];

        for (let part of json_parts) {
            try {
                let payload = JSON.parse(part);
                let b64_data = payload.data;

                if (b64_data) {
                    try {
                        let decoded = Buffer.from(b64_data, 'base64').toString('utf-8');

                        // Tambahkan ke hasil
                        hasil.push(JSON.parse(decoded));

                        // TODO: bisa diubah untuk kirim ke ThingsBoard / Home Assistant
                    } catch (decodeError) {
                        node.error("Gagal decode base64: " + decodeError.toString());
                        hasil.push({ error: "Gagal decode base64", detail: decodeError.toString() });
                    }
                } else {
                    node.warn("Field 'data' tidak ditemukan.");
                    hasil.push({ loging: "Field 'data' tidak ditemukan." });
                }
            } catch (jsonError) {
                node.error("Gagal parsing JSON: " + jsonError.toString());
                hasil.push({ error: "Gagal parsing JSON", detail: jsonError.toString() });
            }
        }

        node.warn(JSON.stringify(hasil));
        msg.payload = hasil;
        return msg;

    } catch (e) {
        msg.payload = { error: "Gagal memproses payload", detail: e.toString() };
        return msg;
    }

} else {
    msg.payload = { error: "Payload kosong." };
    return msg;
}