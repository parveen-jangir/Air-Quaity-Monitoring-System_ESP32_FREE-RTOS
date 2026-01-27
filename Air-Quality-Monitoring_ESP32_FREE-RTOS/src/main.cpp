#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "Adafruit_SHT31.h"
#include "Adafruit_SGP40.h"

/* -------------------- DWIN UART CONFIG -------------------- */
#define DWIN_UART Serial2
#define DWIN_RX_PIN 16
#define DWIN_TX_PIN 17
#define DWIN_BAUD   115200

#define DWIN_MAX_FRAME 256

/* -------------------- GLOBALS -------------------- */
SemaphoreHandle_t dwinUartMutex;

/* -------------------- SENSORS -------------------- */
Adafruit_SHT31 sht31;
Adafruit_SGP40 sgp;
bool sht31_ok = false;
bool sgp_ok = false;

/* -------------------- VP DATA TYPE -------------------- */
typedef enum {
  VP_TYPE_INT,
  VP_TYPE_STRING
} vp_type_t;

/* -------------------- VP TYPE MAP -------------------- */
vp_type_t getVpType(uint16_t vp) {
  switch (vp) {
    // Data VPs (Text/ASCII)
    case 0x5012:
    case 0x5022:
    case 0x5032:
    case 0x5042:
      return VP_TYPE_STRING;

    // Settings VPs (INT)
    case 0x5110:  // Temp unit
    case 0x5111:  // Temp enable
    case 0x5120:  // Pressure unit
    case 0x5121:  // Pressure enable
    case 0x5131:  // Humidity enable
    case 0x5141:  // AQI enable
      return VP_TYPE_INT;

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
  frame[2] = 3 + length;
  frame[3] = 0x82;
  frame[4] = vp >> 8;
  frame[5] = vp & 0xFF;
  memcpy(&frame[6], data, length);

  if (xSemaphoreTake(dwinUartMutex, portMAX_DELAY) == pdTRUE) {
    DWIN_UART.write(frame, sizeof(frame));
    DWIN_UART.flush();
    xSemaphoreGive(dwinUartMutex);
  }
}

/* -------------------- HELPER FUNCTIONS -------------------- */
void dwinSendVP_u16(uint16_t vp, uint16_t value) {
  uint8_t data[2] = {(uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
  dwinSendVP(vp, data, 2);
}

/* -------------------- DWIN READ VP -------------------- */
void dwinReadVP(uint16_t vp, uint8_t wordCount) {
  if (wordCount == 0 || wordCount > 120) return;

  uint8_t frame[7] = {
    0x5A, 0xA5, 4,
    0x83,
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
  {0x5010, 0x5011, 0x5012, 0x5019},  // A (Top)
  {0x5020, 0x5021, 0x5022, 0x5029},  // B
  {0x5030, 0x5031, 0x5032, 0x5039},  // C
  {0x5040, 0x5041, 0x5042, 0x5049}   // D (Bottom)
};

/*
 * Set a widget.
 * data_str: data string (up to 4 chars), or NULL to fully clear data (all 0xFF)
 */
void setWidget(uint8_t index, ParameterType param, UnitType unit, const char *data_str = "NaN", uint16_t alarm = 1) {
  if (index >= 4) return;

  const WidgetVPs &w = widgets[index];

  dwinSendVP_u16(w.typeVp, param);
  dwinSendVP_u16(w.unitVp, unit);

  uint8_t dataBytes[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  if (data_str != NULL) {
    size_t len = strlen(data_str);
    if (len > 4) len = 4;
    memcpy(dataBytes, data_str, len);
  }
  dwinSendVP(w.dataVp, dataBytes, 4);

  dwinSendVP_u16(w.alarmVp, alarm);
}

void clearWidget(uint8_t index) {
  setWidget(index, PARAM_BLANK, UNIT_BLANK, NULL, 1);  // NULL → all 0xFF
}

/* -------------------- STATE VARIABLES -------------------- */
bool tempEnabled = false;
bool pressureEnabled = false;
bool humidityEnabled = false;
bool aqiEnabled = false;

UnitType tempUnit = UNIT_F;         // Default °F
UnitType pressureUnit = UNIT_MMH2O; // Default mm H₂O

ParameterType shownInWidget[4] = {PARAM_BLANK, PARAM_BLANK, PARAM_BLANK, PARAM_BLANK};

/* -------------------- DASHBOARD REFRESH -------------------- */
void refreshDashboard() {
  ParameterType enabledParams[4];
  uint8_t count = 0;

  if (tempEnabled) enabledParams[count++] = PARAM_TEMPERATURE;
  if (pressureEnabled) enabledParams[count++] = PARAM_PRESSURE;
  if (humidityEnabled) enabledParams[count++] = PARAM_HUMIDITY;
  if (aqiEnabled) enabledParams[count++] = PARAM_AQI;

  for (uint8_t i = 0; i < 4; i++) {
    if (i < count) {
      ParameterType param = enabledParams[i];
      UnitType unit;

      switch (param) {
        case PARAM_TEMPERATURE: unit = tempUnit; break;
        case PARAM_PRESSURE:    unit = pressureUnit; break;
        case PARAM_HUMIDITY:    unit = UNIT_RH; break;
        case PARAM_AQI:         unit = UNIT_BLANK; break;
        default:                unit = UNIT_BLANK;
      }

      setWidget(i, param, unit, "NaN");  // Initial "NaN" until refresh
      shownInWidget[i] = param;
    } else {
      clearWidget(i);
      shownInWidget[i] = PARAM_BLANK;
    }
  }
}

/* -------------------- SENSOR READING & DATA REFRESH TASK -------------------- */
void dwinDataRefreshTask(void *pvParameters) {
  while (1) {
    // Default to NaN
    String temp_str = "NaN";
    String hum_str  = "NaN";
    String aqi_str  = "NaN";
    String pres_str = "NaN";  // No pressure sensor

    float t = NAN, h = NAN;
    bool valid_temp_hum = false;

    if (sht31_ok) {
      t = sht31.readTemperature();
      h = sht31.readHumidity();
      if (!isnan(t) && !isnan(h)) {
        valid_temp_hum = true;

        // Temperature (with unit conversion)
        int temp_int = round(t);
        if (tempUnit == UNIT_F) {
          temp_int = round(t * 1.8f + 32.0f);
        }
        temp_str = String(temp_int);

        // Humidity
        hum_str = String((int)round(h));
      }
    }

    // AQI (VOC Index) - only if temp/hum valid and SGP40 ok
    if (valid_temp_hum && sgp_ok) {
      int32_t voc = sgp.measureVocIndex(t, h);
      aqi_str = String(voc);
    }

    // Send to active widgets
    for (uint8_t i = 0; i < 4; i++) {
      if (shownInWidget[i] == PARAM_BLANK) continue;

      String data_str;
      switch (shownInWidget[i]) {
        case PARAM_TEMPERATURE: data_str = temp_str; break;
        case PARAM_PRESSURE:    data_str = pres_str; break;
        case PARAM_HUMIDITY:    data_str = hum_str;  break;
        case PARAM_AQI:         data_str = aqi_str;  break;
        default: continue;
      }

      uint8_t dataBytes[4] = {0xFF, 0xFF, 0xFF, 0xFF};
      size_t len = data_str.length();
      if (len > 4) len = 4;
      for (size_t j = 0; j < len; j++) {
        dataBytes[j] = data_str.charAt(j);
      }
      dwinSendVP(widgets[i].dataVp, dataBytes, 4);
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

/* ----------- Application handlers ----------- */
void handleIntVP(uint16_t vp, uint16_t value) {
  Serial.printf("INT  VP 0x%04X = %d\n", vp, value);

  bool changed = false;

  switch (vp) {
    case 0x5110:  // Temp unit
      tempUnit = (value == 0) ? UNIT_F : UNIT_C;
      changed = true;
      break;
    case 0x5111:  // Temp enable
      tempEnabled = (value == 0);
      changed = true;
      break;
    case 0x5120:  // Pressure unit
      pressureUnit = (value == 0) ? UNIT_MMH2O : UNIT_PA;
      changed = true;
      break;
    case 0x5121:  // Pressure enable
      pressureEnabled = (value == 0);
      changed = true;
      break;
    case 0x5131:  // Humidity enable
      humidityEnabled = (value == 0);
      changed = true;
      break;
    case 0x5141:  // AQI enable
      aqiEnabled = (value == 0);
      changed = true;
      break;
  }

  if (changed) {
    refreshDashboard();
  }
}

void handleStringVP(uint16_t vp, const char *text) {
  Serial.printf("STR  VP 0x%04X = %s\n", vp, text);
}

/* ----------- Frame Parser ----------- */
void parseDwinFrame(uint8_t *frame, uint16_t len) {
  if (len < 7 || frame[3] != 0x83) return;

  uint16_t vp = (frame[4] << 8) | frame[5];
  uint8_t wordCount = frame[6];
  uint8_t *data = &frame[7];

  vp_type_t type = getVpType(vp);

  if (type == VP_TYPE_INT && wordCount >= 1) {
    uint16_t value = (data[0] << 8) | data[1];
    handleIntVP(vp, value);
  } else if (type == VP_TYPE_STRING) {
    char text[256] = {0};
    uint8_t maxBytes = wordCount * 2;
    uint8_t i = 0;
    while (i < maxBytes && i < sizeof(text) - 1 && data[i] != 0xFF) {
      text[i] = data[i];
      i++;
    }
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
      if (index == 1 && byteIn != 0xA5) { index = 0; continue; }

      buffer[index++] = byteIn;

      if (index == 3) expectedLen = buffer[2] + 3;

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
  Serial.println("ESP32 + FreeRTOS + DWIN + Sensors starting...");

  Wire.begin(21, 22);  // SDA, SCL

  sht31_ok = sht31.begin(0x44);
  if (!sht31_ok) Serial.println("SHT31 not found");

  sgp_ok = sgp.begin();
  if (!sgp_ok) Serial.println("SGP40 not found");

  DWIN_UART.begin(DWIN_BAUD, SERIAL_8N1, DWIN_RX_PIN, DWIN_TX_PIN);

  dwinUartMutex = xSemaphoreCreateMutex();
  if (dwinUartMutex == NULL) {
    Serial.println("Failed to create UART mutex");
    while (1);
  }

  xTaskCreate(dwinRxTask, "DWIN_RX", 4096, NULL, 2, NULL);
  xTaskCreate(dwinDataRefreshTask, "DWIN_REFRESH", 4096, NULL, 2, NULL);

  // Initial state sync
  dwinReadVP(0x5110, 1);
  dwinReadVP(0x5111, 1);
  dwinReadVP(0x5120, 1);
  dwinReadVP(0x5121, 1);
  dwinReadVP(0x5131, 1);
  dwinReadVP(0x5141, 1);
}

/* -------------------- ARDUINO LOOP -------------------- */
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}