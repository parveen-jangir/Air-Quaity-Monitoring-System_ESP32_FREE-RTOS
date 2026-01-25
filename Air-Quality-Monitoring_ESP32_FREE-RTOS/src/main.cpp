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
    // Data VPs (Text/ASCII)
    case 0x5012:
    case 0x5022:
    case 0x5032:
    case 0x5042:
      return VP_TYPE_STRING;

    // Examples from original code
    case 0x5121:   // integer example
      return VP_TYPE_INT;

    case 0x5115:   // string example
    case 0x5135:   // string example
    case 0x5145:   // string example
    case 0x5125:   // string example
    case 0x6050:   // string example
      return VP_TYPE_STRING;

    // All others (icons, units, alarms) default to INT
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

/* -------------------- DWIN READ VP -------------------- */
void dwinReadVP(uint16_t vp, uint8_t wordCount) {
  if (wordCount == 0 || wordCount > 120) return;  // Limit per protocol

  uint8_t frame[7] = {
    0x5A, 0xA5, 4,      // LEN = CMD + VP(2) + COUNT
    0x83,               // Read VP
    vp >> 8, vp & 0xFF,
    wordCount
  };

  if (xSemaphoreTake(dwinUartMutex, portMAX_DELAY) == pdTRUE) {
    DWIN_UART.write(frame, sizeof(frame));
    DWIN_UART.flush();
    xSemaphoreGive(dwinUartMutex);
  }
}

/* -------------------- PAGE 8 WIDGET CONFIG -------------------- */
typedef enum {
  PARAM_BLANK = 0,
  PARAM_PRESSURE = 1,
  PARAM_TEMPERATURE = 2,
  PARAM_HUMIDITY = 3,
  PARAM_AQI = 4
} ParameterType;

typedef enum {
  UNIT_F = 0,
  UNIT_C = 1,
  UNIT_MMH2O = 2,
  UNIT_RH = 3,
  UNIT_PA = 4,
  UNIT_BLANK = 5
} UnitType;

struct WidgetVPs {
  uint16_t typeVp;
  uint16_t unitVp;
  uint16_t dataVp;
  uint16_t alarmVp;
};

const WidgetVPs widgets[4] = {
  {0x5010, 0x5011, 0x5012, 0x5019},  // Widget A (Top)
  {0x5020, 0x5021, 0x5022, 0x5029},  // Widget B
  {0x5030, 0x5031, 0x5032, 0x5039},  // Widget C
  {0x5040, 0x5041, 0x5042, 0x5049}   // Widget D (Bottom)
};

/*
 * Set a widget on Page 8.
 * - index: 0 (A/Top) to 3 (D/Bottom)
 * - param: ParameterType (icon)
 * - unit: UnitType
 * - data: ASCII string (up to 4 chars) or NULL to clear data
 * - alarm: Alarm state (default 1 = normal/no border; discuss later for other values like blink)
 *
 * Always sends 4 bytes for data: ASCII chars padded with 0xFF, or all 0xFF if clearing.
 */
void setWidget(uint8_t index, ParameterType param, UnitType unit, const char *data, uint16_t alarm = 1) {
  if (index >= 4) return;

  const WidgetVPs &w = widgets[index];

  dwinSendVP_u16(w.typeVp, param);
  dwinSendVP_u16(w.unitVp, unit);

  // Prepare data: always 4 bytes, pad with 0xFF
  uint8_t dataBytes[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  if (data != NULL) {
    size_t len = strlen(data);
    if (len > 4) len = 4;
    memcpy(dataBytes, data, len);
    // Remaining bytes stay 0xFF
  }
  dwinSendVP(w.dataVp, dataBytes, 4);

  dwinSendVP_u16(w.alarmVp, alarm);
}

/*
 * Clear (hide) a widget fully.
 * Equivalent to setWidget(index, PARAM_BLANK, UNIT_BLANK, NULL, 1)
 */
void clearWidget(uint8_t index) {
  setWidget(index, PARAM_BLANK, UNIT_BLANK, NULL, 1);
}

/* -------------------- FREERTOS TASK -------------------- */
void dwinTxTask(void *pvParameters) {
  while (1) {
    // Example: Set 4 parameters (adapt to your sensor data)
    // Widget A: Temperature 25 °C, normal alarm
    setWidget(0, PARAM_TEMPERATURE, UNIT_C, "25");

    // Widget B: Pressure 999 mmH2O, normal alarm
    setWidget(1, PARAM_PRESSURE, UNIT_MMH2O, "999");

    // Widget C: Humidity 50 %RH, normal alarm
    setWidget(2, PARAM_HUMIDITY, UNIT_RH, "50");

    // Widget D: AQI 100 (blank unit), normal alarm
    setWidget(3, PARAM_AQI, UNIT_BLANK, "100");

    vTaskDelay(pdMS_TO_TICKS(5000));  // Update every 5s

    // Example: Show only 2 parameters, clear others
    setWidget(0, PARAM_TEMPERATURE, UNIT_F, "77");
    setWidget(1, PARAM_AQI, UNIT_BLANK, "150");
    clearWidget(2);
    clearWidget(3);

    vTaskDelay(pdMS_TO_TICKS(5000));
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
    char text[256];  // Increased size for safety
    uint8_t maxBytes = wordCount * 2;
    uint8_t i = 0;

    while (i < maxBytes && i < sizeof(text) - 1 && data[i] != 0xFF) {
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

  if (xTaskCreate(dwinRxTask, "DWIN_RX", 4096, NULL, 2, NULL) != pdPASS) {
    Serial.println("Failed to create DWIN_RX task");
    while (1);
  }

  if (xTaskCreate(dwinTxTask, "DWIN_TX", 4096, NULL, 2, NULL) != pdPASS) {
    Serial.println("Failed to create DWIN_TX task");
    while (1);
  }
}

/* -------------------- ARDUINO LOOP -------------------- */
void loop() {
  // Not used — FreeRTOS scheduler is running
  vTaskDelay(pdMS_TO_TICKS(1000));
}