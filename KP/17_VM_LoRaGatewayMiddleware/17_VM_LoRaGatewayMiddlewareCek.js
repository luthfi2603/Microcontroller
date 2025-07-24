for (let data of msg.payload) {
    if (data.id === 1) {
        msg.payload = data

        return msg
    }
}

return null