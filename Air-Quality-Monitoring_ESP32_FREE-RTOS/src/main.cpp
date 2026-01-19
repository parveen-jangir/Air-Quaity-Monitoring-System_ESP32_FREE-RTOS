#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* -------------------- DWIN UART CONFIG -------------------- */
#define DWIN_UART Serial2
#define DWIN_RX_PIN 16
#define DWIN_TX_PIN 17
#define DWIN_BAUD   115200

#define DWIN_MAX_FRAME 256

/* -------------------- GLOBALS -------------------- */
SemaphoreHandle_t dwinUartMutex;

/* -------------------- DWIN SEND VP -------------------- */
/*
 * vp     : VP address (2 bytes)
 * data   : pointer to binary payload
 * length : payload length (1–240 bytes)
 */

/* -------------------- VP DATA TYPE -------------------- */
typedef enum {
  VP_TYPE_INT,
  VP_TYPE_STRING
} vp_type_t;

/* -------------------- VP TYPE MAP -------------------- */
/* Decide data type by VP address (recommended) */
vp_type_t getVpType(uint16_t vp) {
  switch (vp) {
    case 0x5121:   // integer example
      return VP_TYPE_INT;

    case 0x5115:   // string example
      return VP_TYPE_STRING;

    case 0x5135:   // string example
      return VP_TYPE_STRING;

    case 0x5145:   // string example
      return VP_TYPE_STRING;

    case 0x5125:   // string example
      return VP_TYPE_STRING;

    case 0x6050:   // string example
      return VP_TYPE_STRING;

    default:
      return VP_TYPE_INT;
  }
}

void dwinSendVP(uint16_t vp, const uint8_t *data, uint8_t length) {
  if (data == NULL || length == 0 || length > 240) {
    return;
  }

  uint8_t frame[6 + length];

  frame[0] = 0x5A;
  frame[1] = 0xA5;
  frame[2] = 3 + length;      // LEN = CMD + VP(2) + DATA
  frame[3] = 0x82;            // Write VP
  frame[4] = vp >> 8;         // VP high
  frame[5] = vp & 0xFF;       // VP low

  memcpy(&frame[6], data, length);

  if (xSemaphoreTake(dwinUartMutex, portMAX_DELAY) == pdTRUE) {
    DWIN_UART.write(frame, sizeof(frame));
    DWIN_UART.flush();
    xSemaphoreGive(dwinUartMutex);
  }
}

/* -------------------- HELPER FUNCTIONS -------------------- */
void dwinSendVP_u8(uint16_t vp, uint8_t value) {
  dwinSendVP(vp, &value, 1);
}

void dwinSendVP_u16(uint16_t vp, uint16_t value) {
  uint8_t data[2] = {
    (uint8_t)(value >> 8),
    (uint8_t)(value & 0xFF)
  };
  dwinSendVP(vp, data, 2);
}

void dwinSendVP_str(uint16_t vp, const char *text) {
  if (text == NULL) return;
  dwinSendVP(vp, (const uint8_t *)text, strlen(text));
}

/* -------------------- FREERTOS TASK -------------------- */
void dwinTxTask(void *pvParameters) {
  uint16_t counter = 0;

  while (1) {
    // dwinSendVP_u16(0x5151, 0x0001);
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // dwinSendVP_u16(0x5151, 0x0000);
    dwinSendVP_str(0x6000, "Pappu ki fauji");

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/* ----------- Application handlers ----------- */
void handleIntVP(uint16_t vp, uint16_t value) {
  Serial.printf("INT  VP 0x%04X = %d\n", vp, value);
}

void handleStringVP(uint16_t vp, const char *text) {
  Serial.printf("STR  VP 0x%04X = %s\n", vp, text);
}

/* ----------- Frame Parser ----------- */
void parseDwinFrame(uint8_t *frame, uint16_t len) {
  if (len < 7) return;
  if (frame[3] != 0x83) return;   // Not a read response

  uint16_t vp = (frame[4] << 8) | frame[5];
  uint8_t wordCount = frame[6];
  uint8_t *data = &frame[7];

  vp_type_t type = getVpType(vp);

  if (type == VP_TYPE_INT) {
    if (wordCount >= 1) {
      uint16_t value = (data[0] << 8) | data[1];
      handleIntVP(vp, value);
    }
  }
  else if (type == VP_TYPE_STRING) {
    char text[64];
    uint8_t maxBytes = wordCount * 2;
    uint8_t i = 0;

    while (i < maxBytes && data[i] != 0xFF) {
      text[i] = data[i];
      i++;
    }
    text[i] = '\0';

    handleStringVP(vp, text);
  }
}

void dwinRxTask(void *pvParameters) {
  uint8_t buffer[DWIN_MAX_FRAME];
  uint16_t index = 0;
  uint16_t expectedLen = 0;

  while (1) {
    while (DWIN_UART.available()) {
      uint8_t byteIn = DWIN_UART.read();

      if (index == 0 && byteIn != 0x5A) continue;
      if (index == 1 && byteIn != 0xA5) {
        index = 0;
        continue;
      }

      buffer[index++] = byteIn;

      if (index == 3) {
        expectedLen = buffer[2] + 3;
      }

      if (expectedLen && index >= expectedLen) {
        parseDwinFrame(buffer, expectedLen);
        index = 0;
        expectedLen = 0;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/* -------------------- ARDUINO SETUP -------------------- */
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("ESP32 + FreeRTOS + DWIN DGUS starting...");

  DWIN_UART.begin(
    DWIN_BAUD,
    SERIAL_8N1,
    DWIN_RX_PIN,
    DWIN_TX_PIN
  );

  dwinUartMutex = xSemaphoreCreateMutex();
  if (dwinUartMutex == NULL) {
    Serial.println("Failed to create UART mutex");
    while (1);
  }
  dwinUartMutex = xSemaphoreCreateMutex();
  if (!dwinUartMutex) {
    Serial.println("UART mutex creation failed");
    while (1);
  }

  xTaskCreate(dwinRxTask, "DWIN_RX", 4096, NULL, 2, NULL);
  // xTaskCreate(dwinTxTask, "DWIN_TX", 4096, NULL, 2, NULL);

}

/* -------------------- ARDUINO LOOP -------------------- */
void loop() {
  // Not used — FreeRTOS scheduler is running
  vTaskDelay(pdMS_TO_TICKS(1000));
}
