/* Heltec Automation example
 * HelTec AutoMation, Chengdu, China
 * www.heltec.org
 *
 * this project also release in GitHub:
 * https://github.com/HelTecAutomation/ASR650x-Arduino
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"

/* if defined  SENDER, the node work as sender
 * the node will send data {0x00,0x01,0x02,0x03} every 2 seconds
 * subscribe UPlink Topic(like "GWID/60FB00FFFE63A3E1/UP" ) will get message like this.
 * {"freq":486.300000,"datr":"SF7BW125","rssis":-9,"lsnr":12.8,"rssi":-10,"size":4,"data":"AAECAw=="}
 * The freq "486.3" is based on RF_FREQUENCY in the code.
 * The data "AAECAw==" is base64 encode of data({0x00,0x01,0x02,0x03}).
 * The datr "SF7BW125" is based on the  LORA_BANDWIDTH and  LORA_SPREADING_FACTOR  in the code.
 */

/* if defined  RECEIVER, the node work as receiver
 * publish a message like {"freq":486.3,"datr":"SF7BW125","size":4,"data":"AAECAw=="}  with Downlink Topic(like "GWID/60FB00FFFE63A3E1/DOWN" )
 *    The freq "486.3" is based on RF_FREQUENCY in the code.
 *    The data "AAECAw==" is base64 encode of data({0x00,0x01,0x02,0x03}).
 *    The datr "SF7BW125" is based on the  LORA_BANDWIDTH and  LORA_SPREADING_FACTOR  in the code.
 * the node will recive data {0x00,0x01,0x02,0x03} and print it on the UART Serial.
 */
// #define SENDER
#define RECEIVER

#define RF_FREQUENCY 486300000  // Hz
#define TX_OUTPUT_POWER 22      // dBm
#define LORA_BANDWIDTH 0        // [0: 125 kHz,
                                //  1: 250 kHz,
                                //  2: 500 kHz,
                                //  3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]

uint8_t data[] = {0x00, 0x01, 0x02, 0x03};

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

typedef enum {
  LOWPOWER,
  RX,
  TX
} States_t;

States_t state;

void setup() {
  Serial.begin(115200);

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  #ifdef SENDER
    state = TX;
  #endif

  #ifdef RECEIVER
    state = RX;
  #endif
}

void loop() {
  switch (state) {
    case TX:
      delay(2000);
      Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, 1, 8, false, true, 0, 0, false, 3000);
      Radio.Send(data, 4);
      state = LOWPOWER;
      break;
    case RX:
      Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, 1, 0, 8, 0, false, 0, false, 0, 0, true, true);
      Radio.Rx(0);
      state = LOWPOWER;
      break;
    case LOWPOWER:
      lowPowerHandler();
      break;
    default:
      break;
  }
}

void OnTxDone(void) {
  Serial.print("TX done......");
  state = TX;
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.print("TX Timeout......");
  state = TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  Serial.printf("rececived %d bytes with rssi %d,snr%d\r\n", size, rssi, snr);
  Serial.printf("data:");

  for (int i = 0; i < size; i++) {
    Serial.printf("0x%02X ", payload[i]);
  }
  
  Serial.printf("\r\n");
  delay(10);
}