// Mengecek apakah payload ada
if (msg.payload) {
    try {
        // Fungsi untuk memisahkan objek JSON dalam satu string panjang (mirip split_json_objects)
        let jsonParts = msg.payload.match(/\{.*?\}/g);

        if (!jsonParts || jsonParts.length === 0) {
            node.warn("Tidak ditemukan objek JSON dalam payload!");
            msg.payload = {error: "Tidak ditemukan objek JSON dalam payload!"};

            return msg;
        }

        // Array untuk menyimpan hasil decode
        let result = [];

        for (let part of jsonParts) {
            try {
                let payload = JSON.parse(part);
                let b64_data = payload.data;

                if (b64_data) {
                    try {
                        let decoded = Buffer.from(b64_data, 'base64').toString('utf-8');

                        // Tambahkan ke hasil
                        result.push(JSON.parse(decoded));
                    } catch (decodeError) {
                        node.error("Gagal decode base64: " + decodeError.toString());
                        result.push({error: "Gagal decode base64", detail: decodeError.toString()});
                    }
                } else {
                    node.warn("Field 'data' tidak ditemukan!");
                    result.push({ loging: "Field 'data' tidak ditemukan!" });
                }
            } catch (jsonError) {
                node.error("Gagal parsing JSON: " + jsonError.toString());
                result.push({error: "Gagal parsing JSON", detail: jsonError.toString()});
            }
        }

        node.warn(JSON.stringify(result));
        msg.payload = result;

        return msg;
    } catch (e) {
        msg.payload = {error: "Gagal memproses payload", detail: e.toString()};

        return msg;
    }
} else {
    msg.payload = {error: "Payload kosong!"};

    return msg;
}