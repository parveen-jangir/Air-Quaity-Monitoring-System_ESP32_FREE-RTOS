#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "Adafruit_SHT31.h"
#include "Adafruit_SGP40.h"
#include <ModbusRTU.h>
#include <SoftwareSerial.h>

/* -------------------- MODBUS CONFIG -------------------- */
#define MODBUS_TX_PIN     14
#define MODBUS_RX_PIN     13
#define MODBUS_DE_PIN     4

#define MODBUS_DEFAULT_SLAVE_ID   1
#define MODBUS_DEFAULT_BAUD       9600

#define MODBUS_DISABLED_VALUE     -9999

// MODBUS Register Addresses - Holding Registers (R/W)
#define HREG_TEMP_ENABLE      0
#define HREG_HUMIDITY_ENABLE  1
#define HREG_PRESSURE_ENABLE  2
#define HREG_AQI_ENABLE       3
#define HREG_TEMP_OFFSET      4
#define HREG_HUMIDITY_OFFSET  5
#define HREG_PRESSURE_OFFSET  6
#define HREG_AQI_OFFSET       7
#define HREG_ALARM_ACK        8
#define HREG_COUNT            9

// MODBUS Register Addresses - Input Registers (R)
#define IREG_TEMP_VALUE       0
#define IREG_HUMIDITY_VALUE   1
#define IREG_PRESSURE_VALUE   2
#define IREG_AQI_VALUE        3
#define IREG_TEMP_UNIT        4
#define IREG_PRESSURE_UNIT    5
#define IREG_COUNT            6

// MODBUS Settings VPs
#define VP_MODBUS_ENABLE      0x5500
#define VP_MODBUS_SLAVE_ID    0x5510
#define VP_MODBUS_BAUD_RATE   0x5530

/* -------------------- DWIN UART CONFIG -------------------- */
#define DWIN_UART Serial2
#define DWIN_RX_PIN 16
#define DWIN_TX_PIN 17
#define DWIN_BAUD   115200

#define DWIN_MAX_FRAME 256

/* -------------------- MODBUS GLOBALS -------------------- */
SoftwareSerial modbusSerial(MODBUS_RX_PIN, MODBUS_TX_PIN);
ModbusRTU mb;

bool modbusEnabled = true;          // Default: ON
uint8_t modbusSlaveId = MODBUS_DEFAULT_SLAVE_ID;
uint32_t modbusBaudRate = MODBUS_DEFAULT_BAUD;

// Flag to track if MODBUS needs reinitialization
bool modbusNeedsReinit = false;
bool modbusInternalWrite = false;

/* -------------------- GLOBALS -------------------- */
SemaphoreHandle_t dwinUartMutex;

/* -------------------- EEPROM / PREFERENCES -------------------- */
Preferences preferences;

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

/* -------------------- CALIBRATION & LIMITS STRUCTURE -------------------- */
typedef struct {
  int16_t offset;           // Calibration offset
  int16_t upperLimit;       // Upper limit for alarm
  int16_t lowerLimit;       // Lower limit for alarm
  bool upperLimitSet;       // Is upper limit configured (not "-")
  bool lowerLimitSet;       // Is lower limit configured (not "-")
  bool alarmActive;         // Is alarm currently triggered (value out of limits)
  bool alarmAcknowledged;   // Has alarm been acknowledged by user
  bool dataValid;           // Is sensor data valid (not NaN)
} ParameterConfig;

// Configuration for each parameter (with default values - limits not set)
ParameterConfig tempConfig     = {0, 0, 0, false, false, false, false, false};
ParameterConfig humidityConfig = {0, 0, 0, false, false, false, false, false};
ParameterConfig pressureConfig = {0, 0, 0, false, false, false, false, false};
ParameterConfig aqiConfig      = {0, 0, 0, false, false, false, false, false};

// Current calibrated values (updated in data refresh task)
int currentTempValue = 0;
int currentHumidityValue = 0;
int currentPressureValue = 0;
int currentAqiValue = 0;

// Alarm blink state (toggles every 500ms)
bool alarmBlinkState = false;

// Buzzer sound setting (default: ON)
bool buzzerEnabled = true;

/* -------------------- DATE & TIME VARIABLES -------------------- */
uint8_t rtcYear   = 26;   // 2026 (offset from 2000)
uint8_t rtcMonth  = 1;    // January
uint8_t rtcDay    = 1;    // 1st
uint8_t rtcHour   = 0;    // 00:xx
uint8_t rtcMinute = 0;    // xx:00
uint8_t rtcSecond = 0;    // xx:xx:00

// Mutex for RTC variables (accessed by multiple tasks)
SemaphoreHandle_t rtcMutex;

/* -------------------- PASSWORD CONFIGURATION -------------------- */
#define DEFAULT_SETTINGS_PASSWORD   "123456"
#define ADVANCED_SETTINGS_PASSWORD  "124578"
#define PASSWORD_LENGTH             6

// Settings password (stored in EEPROM, changeable)
char settingsPassword[PASSWORD_LENGTH + 1] = DEFAULT_SETTINGS_PASSWORD;

// Page IDs
#define PAGE_SETTINGS          0x01
#define PAGE_ADVANCED_SETTINGS 0x0A

/* -------------------- VP ADDRESSES -------------------- */
// DWIN RTC Register
#define VP_DWIN_RTC             0x0010

// Calibration Offset VPs (ASCII, 4 bytes)
#define VP_TEMP_CALIBRATION     0x5215
#define VP_HUMIDITY_CALIBRATION 0x5225
#define VP_PRESSURE_CALIBRATION 0x5235
#define VP_AQI_CALIBRATION      0x5245

// Upper Limit VPs (ASCII, 4 bytes)
#define VP_TEMP_UPPER_LIMIT     0x5112
#define VP_HUMIDITY_UPPER_LIMIT 0x5122
#define VP_PRESSURE_UPPER_LIMIT 0x5132
#define VP_AQI_UPPER_LIMIT      0x5142

// Lower Limit VPs (ASCII, 4 bytes)
#define VP_TEMP_LOWER_LIMIT     0x5115
#define VP_HUMIDITY_LOWER_LIMIT 0x5125
#define VP_PRESSURE_LOWER_LIMIT 0x5135
#define VP_AQI_LOWER_LIMIT      0x5145

// Buzzer register address
#define DWIN_BUZZER_VP          0x00A0
#define DWIN_BUZZER_ON          0x000F

// Password VPs (ASCII, 6 bytes)
#define VP_SETTINGS_PASSWORD_ENTRY    0x5350
#define VP_SETTINGS_PASSWORD_CHANGE   0x5360
#define VP_ADVANCED_PASSWORD_ENTRY    0x5370

// Alarm Acknowledge VP
#define VP_ALARM_ACK            0x5001

// Buzzer Sound Setting VP
#define VP_BUZZER_SETTING       0x5151

// Date & Time Input VPs (ASCII)
#define VP_DATE_INPUT           0x5410
#define VP_TIME_INPUT           0x5430

/* -------------------- VP TYPE MAP -------------------- */
vp_type_t getVpType(uint16_t vp) {
  switch (vp) {
    // Data VPs (Text/ASCII)
    case 0x5012:
    case 0x5022:
    case 0x5032:
    case 0x5042:
    // Calibration VPs (Text/ASCII)
    case VP_TEMP_CALIBRATION:
    case VP_HUMIDITY_CALIBRATION:
    case VP_PRESSURE_CALIBRATION:
    case VP_AQI_CALIBRATION:
    // Upper Limit VPs (Text/ASCII)
    case VP_TEMP_UPPER_LIMIT:
    case VP_HUMIDITY_UPPER_LIMIT:
    case VP_PRESSURE_UPPER_LIMIT:
    case VP_AQI_UPPER_LIMIT:
    // Lower Limit VPs (Text/ASCII)
    case VP_TEMP_LOWER_LIMIT:
    case VP_HUMIDITY_LOWER_LIMIT:
    case VP_PRESSURE_LOWER_LIMIT:
    case VP_AQI_LOWER_LIMIT:
    // Password VPs (Text/ASCII)
    case VP_SETTINGS_PASSWORD_ENTRY:
    case VP_SETTINGS_PASSWORD_CHANGE:
    case VP_ADVANCED_PASSWORD_ENTRY:
    // Date & Time VPs (Text/ASCII)
    case VP_DATE_INPUT:
    case VP_TIME_INPUT:
    case VP_MODBUS_SLAVE_ID:
    case VP_MODBUS_BAUD_RATE:
      return VP_TYPE_STRING;

    // MODBUS Enable VP (INT)
    
    // Settings VPs (INT)
    case 0x5110:  // Temp unit
    case 0x5111:  // Temp enable
    case 0x5120:  // Pressure unit
    case 0x5121:  // Pressure enable
    case 0x5131:  // Humidity enable
    case 0x5141:  // AQI enable
    case VP_ALARM_ACK:      // Alarm acknowledge button
    case VP_BUZZER_SETTING: // Buzzer sound on/off
    case VP_MODBUS_ENABLE:
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

/* -------------------- DWIN SWITCH PAGE -------------------- */
void dwinSwitchPage(uint8_t pageId) {
  // Command format: 5A A5 07 82 00 84 5A 01 00 XX (XX = page ID)
  uint8_t frame[10] = {
    0x5A, 0xA5, 0x07,
    0x82,
    0x00, 0x84,
    0x5A, 0x01,
    0x00, pageId
  };

  if (xSemaphoreTake(dwinUartMutex, portMAX_DELAY) == pdTRUE) {
    DWIN_UART.write(frame, sizeof(frame));
    DWIN_UART.flush();
    xSemaphoreGive(dwinUartMutex);
  }

  Serial.printf("[PAGE] Switched to page %d\n", pageId);
}

/* -------------------- DWIN READ VP -------------------- */
void dwinReadVP(uint16_t vp, uint8_t wordCount) {
  if (wordCount == 0 || wordCount > 120) return;

  uint8_t frame[7] = {
    0x5A, 0xA5, 4,
    0x83,
    (uint8_t)(vp >> 8), (uint8_t)(vp & 0xFF),
    wordCount
  };

  if (xSemaphoreTake(dwinUartMutex, portMAX_DELAY) == pdTRUE) {
    DWIN_UART.write(frame, sizeof(frame));
    DWIN_UART.flush();
    xSemaphoreGive(dwinUartMutex);
  }
}

/* -------------------- DATE & TIME FUNCTIONS -------------------- */

// Check if year is leap year (year is offset from 2000, e.g., 26 = 2026)
bool isLeapYear(uint8_t year) {
  uint16_t fullYear = 2000 + year;
  return (fullYear % 4 == 0 && fullYear % 100 != 0) || (fullYear % 400 == 0);
}

// Get days in month
uint8_t getDaysInMonth(uint8_t month, uint8_t year) {
  switch (month) {
    case 1:  return 31;  // January
    case 2:  return isLeapYear(year) ? 29 : 28;  // February
    case 3:  return 31;  // March
    case 4:  return 30;  // April
    case 5:  return 31;  // May
    case 6:  return 30;  // June
    case 7:  return 31;  // July
    case 8:  return 31;  // August
    case 9:  return 30;  // September
    case 10: return 31;  // October
    case 11: return 30;  // November
    case 12: return 31;  // December
    default: return 31;
  }
}

// Send current date/time to DWIN RTC register
void dwinUpdateRTC() {
  // Command: 5A A5 0B 82 00 10 [YY] [MM] [DD] [WW] [HH] [mm] [ss] 00
  uint8_t frame[14] = {
    0x5A, 0xA5, 0x0B,
    0x82,
    0x00, 0x10,
    rtcYear,
    rtcMonth,
    rtcDay,
    0x00,       // Week day (not used)
    rtcHour,
    rtcMinute,
    rtcSecond,
    0x00        // End byte
  };

  if (xSemaphoreTake(dwinUartMutex, portMAX_DELAY) == pdTRUE) {
    DWIN_UART.write(frame, sizeof(frame));
    DWIN_UART.flush();
    xSemaphoreGive(dwinUartMutex);
  }
}

// Parse date string "DD/MM/YYYY" and update RTC
bool parseDateInput(const char *text) {
  if (text == NULL || strlen(text) < 10) return false;

  // Expected format: DD/MM/YYYY (e.g., "01/02/2026")
  int day, month, year;
  
  if (sscanf(text, "%d/%d/%d", &day, &month, &year) != 3) {
    Serial.println("[RTC] Failed to parse date format");
    return false;
  }

  // Validate ranges
  if (year < 2000 || year > 2099) {
    Serial.printf("[RTC] Invalid year: %d\n", year);
    return false;
  }
  if (month < 1 || month > 12) {
    Serial.printf("[RTC] Invalid month: %d\n", month);
    return false;
  }
  
  uint8_t yearOffset = year - 2000;
  uint8_t maxDays = getDaysInMonth(month, yearOffset);
  
  if (day < 1 || day > maxDays) {
    Serial.printf("[RTC] Invalid day: %d (max %d for month %d)\n", day, maxDays, month);
    return false;
  }

  // Update RTC variables with mutex protection
  if (xSemaphoreTake(rtcMutex, portMAX_DELAY) == pdTRUE) {
    rtcYear = yearOffset;
    rtcMonth = month;
    rtcDay = day;
    xSemaphoreGive(rtcMutex);
  }

  Serial.printf("[RTC] Date set to: %02d/%02d/%04d\n", rtcDay, rtcMonth, 2000 + rtcYear);
  
  // Immediately update display
  dwinUpdateRTC();
  
  return true;
}

// Parse time string "HH;MM" and update RTC
bool parseTimeInput(const char *text) {
  if (text == NULL || strlen(text) < 5) return false;

  // Expected format: HH;MM (e.g., "01;02")
  int hour, minute;
  
  if (sscanf(text, "%d;%d", &hour, &minute) != 2) {
    Serial.println("[RTC] Failed to parse time format");
    return false;
  }

  // Validate ranges
  if (hour < 0 || hour > 23) {
    Serial.printf("[RTC] Invalid hour: %d\n", hour);
    return false;
  }
  if (minute < 0 || minute > 59) {
    Serial.printf("[RTC] Invalid minute: %d\n", minute);
    return false;
  }

  // Update RTC variables with mutex protection
  if (xSemaphoreTake(rtcMutex, portMAX_DELAY) == pdTRUE) {
    rtcHour = hour;
    rtcMinute = minute;
    rtcSecond = 0;  // Reset seconds to 0
    xSemaphoreGive(rtcMutex);
  }

  Serial.printf("[RTC] Time set to: %02d:%02d:00\n", rtcHour, rtcMinute);
  
  // Immediately update display
  dwinUpdateRTC();
  
  return true;
}

// Increment time by 1 second (called every second)
void rtcTick() {
  if (xSemaphoreTake(rtcMutex, portMAX_DELAY) == pdTRUE) {
    rtcSecond++;
    
    if (rtcSecond >= 60) {
      rtcSecond = 0;
      rtcMinute++;
      
      if (rtcMinute >= 60) {
        rtcMinute = 0;
        rtcHour++;
        
        if (rtcHour >= 24) {
          rtcHour = 0;
          rtcDay++;
          
          uint8_t maxDays = getDaysInMonth(rtcMonth, rtcYear);
          if (rtcDay > maxDays) {
            rtcDay = 1;
            rtcMonth++;
            
            if (rtcMonth > 12) {
              rtcMonth = 1;
              rtcYear++;
              
              // Handle year overflow (99 -> 0)
              if (rtcYear > 99) {
                rtcYear = 0;
              }
            }
          }
        }
      }
    }
    
    xSemaphoreGive(rtcMutex);
  }
}

/* -------------------- RTC TASK -------------------- */
void rtcTask(void *pvParameters) {
  while (1) {
    // Tick every second
    rtcTick();
    
    // Update display every minute (when seconds == 0)
    if (rtcSecond == 0) {
      dwinUpdateRTC();
      Serial.printf("[RTC] %02d/%02d/%04d %02d:%02d:%02d\n", 
                    rtcDay, rtcMonth, 2000 + rtcYear, 
                    rtcHour, rtcMinute, rtcSecond);
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second delay
  }
}

/* -------------------- PASSWORD FUNCTIONS -------------------- */
void loadPasswordFromEEPROM() {
  preferences.begin("settings", true);  // Read-only mode
  String storedPassword = preferences.getString("password", DEFAULT_SETTINGS_PASSWORD);
  preferences.end();

  // Copy to settingsPassword array
  strncpy(settingsPassword, storedPassword.c_str(), PASSWORD_LENGTH);
  settingsPassword[PASSWORD_LENGTH] = '\0';

  Serial.printf("[EEPROM] Loaded settings password: %s\n", settingsPassword);
}

void savePasswordToEEPROM(const char *newPassword) {
  preferences.begin("settings", false);  // Read-write mode
  preferences.putString("password", newPassword);
  preferences.end();

  // Update local copy
  strncpy(settingsPassword, newPassword, PASSWORD_LENGTH);
  settingsPassword[PASSWORD_LENGTH] = '\0';

  Serial.printf("[EEPROM] Saved new settings password: %s\n", settingsPassword);
}

bool validatePassword(const char *enteredPassword, const char *correctPassword) {
  if (enteredPassword == NULL || correctPassword == NULL) return false;
  return (strncmp(enteredPassword, correctPassword, PASSWORD_LENGTH) == 0);
}

bool isValidPasswordFormat(const char *password) {
  if (password == NULL) return false;
  
  size_t len = strlen(password);
  if (len != PASSWORD_LENGTH) return false;
  
  // Check all characters are digits
  for (size_t i = 0; i < len; i++) {
    if (password[i] < '0' || password[i] > '9') return false;
  }
  
  return true;
}

/* -------------------- ALARM ACKNOWLEDGE FUNCTION -------------------- */
void acknowledgeAllAlarms() {
  bool anyAlarmAcknowledged = false;

  // Acknowledge temperature alarm if active
  if (tempConfig.alarmActive && !tempConfig.alarmAcknowledged) {
    tempConfig.alarmAcknowledged = true;
    anyAlarmAcknowledged = true;
    Serial.println("[ACK] Temperature alarm acknowledged");
  }

  // Acknowledge humidity alarm if active
  if (humidityConfig.alarmActive && !humidityConfig.alarmAcknowledged) {
    humidityConfig.alarmAcknowledged = true;
    anyAlarmAcknowledged = true;
    Serial.println("[ACK] Humidity alarm acknowledged");
  }

  // Acknowledge pressure alarm if active
  if (pressureConfig.alarmActive && !pressureConfig.alarmAcknowledged) {
    pressureConfig.alarmAcknowledged = true;
    anyAlarmAcknowledged = true;
    Serial.println("[ACK] Pressure alarm acknowledged");
  }

  // Acknowledge AQI alarm if active
  if (aqiConfig.alarmActive && !aqiConfig.alarmAcknowledged) {
    aqiConfig.alarmAcknowledged = true;
    anyAlarmAcknowledged = true;
    Serial.println("[ACK] AQI alarm acknowledged");
  }

  if (anyAlarmAcknowledged) {
    Serial.println("[ACK] All active alarms acknowledged");
  } else {
    Serial.println("[ACK] No active alarms to acknowledge");
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

/* -------------------- CHECK IF STRING IS VALID NUMBER -------------------- */
/*
 * Returns true if the text represents a valid integer (including negative).
 * Returns false if text is "-", empty, or non-numeric.
 */
bool isValidNumber(const char *text) {
  if (text == NULL || text[0] == '\0') return false;
  
  // Skip leading spaces
  while (*text == ' ') text++;
  
  if (*text == '\0') return false;
  
  // Check for just "-" (not a valid number)
  if (text[0] == '-' && text[1] == '\0') return false;
  
  // Check if it's a valid integer format
  const char *p = text;
  
  // Allow leading minus sign
  if (*p == '-') p++;
  
  // Must have at least one digit
  if (*p == '\0') return false;
  
  // Check all remaining characters are digits
  while (*p != '\0') {
    if (*p < '0' || *p > '9') return false;
    p++;
  }
  
  return true;
}

/* -------------------- PARSE ASCII STRING TO INTEGER -------------------- */
int16_t parseAsciiToInt(const char *text) {
  if (!isValidNumber(text)) return 0;
  return (int16_t)atoi(text);
}

/* -------------------- CHECK ALARM CONDITION -------------------- */
/*
 * Checks if value is out of limits.
 * Also resets alarmAcknowledged flag when value returns to normal.
 * Returns true if value is out of limits, false otherwise.
 */
bool checkAlarmCondition(int value, ParameterConfig *config) {
  // No alarm if data is invalid
  if (!config->dataValid) {
    // Reset acknowledged flag when data is invalid
    config->alarmAcknowledged = false;
    return false;
  }
  
  // No alarm if no limits are set
  if (!config->upperLimitSet && !config->lowerLimitSet) {
    config->alarmAcknowledged = false;
    return false;
  }
  
  bool outOfLimits = false;
  
  // Check upper limit (only if set)
  if (config->upperLimitSet && value > config->upperLimit) {
    outOfLimits = true;
  }
  
  // Check lower limit (only if set)
  if (config->lowerLimitSet && value < config->lowerLimit) {
    outOfLimits = true;
  }
  
  // If value is back within limits, reset the acknowledged flag
  // This allows alarm to trigger again if value goes out of limits
  if (!outOfLimits) {
    config->alarmAcknowledged = false;
  }
  
  return outOfLimits;
}

/* -------------------- GET ALARM STATE FOR PARAMETER -------------------- */
/*
 * Returns true if alarm should be actively showing (blinking + buzzer)
 * Alarm shows only if: alarmActive AND NOT alarmAcknowledged
 */
bool shouldShowAlarm(ParameterType param) {
  switch (param) {
    case PARAM_TEMPERATURE: 
      return tempConfig.alarmActive && !tempConfig.alarmAcknowledged;
    case PARAM_PRESSURE:    
      return pressureConfig.alarmActive && !pressureConfig.alarmAcknowledged;
    case PARAM_HUMIDITY:    
      return humidityConfig.alarmActive && !humidityConfig.alarmAcknowledged;
    case PARAM_AQI:         
      return aqiConfig.alarmActive && !aqiConfig.alarmAcknowledged;
    default:                
      return false;
  }
}

/* -------------------- ALARM BLINK TASK -------------------- */
void alarmBlinkTask(void *pvParameters) {
  while (1) {
    // Toggle blink state
    alarmBlinkState = !alarmBlinkState;
    
    bool anyAlarmShowing = false;
    
    // Check each active widget
    for (uint8_t i = 0; i < 4; i++) {
      ParameterType param = shownInWidget[i];
      
      if (param == PARAM_BLANK) {
        // No parameter in this widget, ensure alarm is off
        dwinSendVP_u16(widgets[i].alarmVp, 1);
        continue;
      }
      
      bool showAlarm = shouldShowAlarm(param);
      
      if (showAlarm) {
        // Alarm active and not acknowledged - blink (0 = red line visible, 1 = no red line)
        uint16_t alarmValue = alarmBlinkState ? 1 : 0;
        dwinSendVP_u16(widgets[i].alarmVp, alarmValue);
        anyAlarmShowing = true;
      } else {
        // Alarm not active or acknowledged - ensure alarm is off (no red line)
        dwinSendVP_u16(widgets[i].alarmVp, 1);
      }
    }
    
    // Sound buzzer only if buzzer is enabled AND any alarm is showing
    if (buzzerEnabled && anyAlarmShowing) {
      dwinSendVP_u16(DWIN_BUZZER_VP, DWIN_BUZZER_ON);
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));  // 500ms blink interval
  }
}

/* -------------------- MODBUS CALLBACK FUNCTIONS -------------------- */

// Callback when Holding Register is written by MODBUS master
uint16_t modbusOnHregSet(TRegister* reg, uint16_t val) {
  if (modbusInternalWrite) {
    return val;
  }
  uint16_t regAddr = reg->address.address;
  
  Serial.printf("[MODBUS] Hreg %d written with value: %d\n", regAddr, val);

  switch (regAddr) {
    case HREG_TEMP_ENABLE:
      tempEnabled = (val == 0);
      // Sync to DWIN display
      dwinSendVP_u16(0x5111, val);
      refreshDashboard();
      Serial.printf("[MODBUS] Temperature %s\n", tempEnabled ? "ENABLED" : "DISABLED");
      break;

    case HREG_HUMIDITY_ENABLE:
      humidityEnabled = (val == 0);
      dwinSendVP_u16(0x5131, val);
      refreshDashboard();
      Serial.printf("[MODBUS] Humidity %s\n", humidityEnabled ? "ENABLED" : "DISABLED");
      break;

    case HREG_PRESSURE_ENABLE:
      pressureEnabled = (val == 0);
      dwinSendVP_u16(0x5121, val);
      refreshDashboard();
      Serial.printf("[MODBUS] Pressure %s\n", pressureEnabled ? "ENABLED" : "DISABLED");
      break;

    case HREG_AQI_ENABLE:
      aqiEnabled = (val == 0);
      dwinSendVP_u16(0x5141, val);
      refreshDashboard();
      Serial.printf("[MODBUS] AQI %s\n", aqiEnabled ? "ENABLED" : "DISABLED");
      break;

    case HREG_TEMP_OFFSET:
      tempConfig.offset = (int16_t)val;
      // Sync to DWIN display (send as ASCII)
      {
        char offsetStr[8];
        snprintf(offsetStr, sizeof(offsetStr), "%d", tempConfig.offset);
        uint8_t dataBytes[4] = {0xFF, 0xFF, 0xFF, 0xFF};
        size_t len = strlen(offsetStr);
        if (len > 4) len = 4;
        memcpy(dataBytes, offsetStr, len);
        dwinSendVP(VP_TEMP_CALIBRATION, dataBytes, 4);
      }
      Serial.printf("[MODBUS] Temp offset set to: %d\n", tempConfig.offset);
      break;

    case HREG_HUMIDITY_OFFSET:
      humidityConfig.offset = (int16_t)val;
      {
        char offsetStr[8];
        snprintf(offsetStr, sizeof(offsetStr), "%d", humidityConfig.offset);
        uint8_t dataBytes[4] = {0xFF, 0xFF, 0xFF, 0xFF};
        size_t len = strlen(offsetStr);
        if (len > 4) len = 4;
        memcpy(dataBytes, offsetStr, len);
        dwinSendVP(VP_HUMIDITY_CALIBRATION, dataBytes, 4);
      }
      Serial.printf("[MODBUS] Humidity offset set to: %d\n", humidityConfig.offset);
      break;

    case HREG_PRESSURE_OFFSET:
      pressureConfig.offset = (int16_t)val;
      {
        char offsetStr[8];
        snprintf(offsetStr, sizeof(offsetStr), "%d", pressureConfig.offset);
        uint8_t dataBytes[4] = {0xFF, 0xFF, 0xFF, 0xFF};
        size_t len = strlen(offsetStr);
        if (len > 4) len = 4;
        memcpy(dataBytes, offsetStr, len);
        dwinSendVP(VP_PRESSURE_CALIBRATION, dataBytes, 4);
      }
      Serial.printf("[MODBUS] Pressure offset set to: %d\n", pressureConfig.offset);
      break;

    case HREG_AQI_OFFSET:
      aqiConfig.offset = (int16_t)val;
      {
        char offsetStr[8];
        snprintf(offsetStr, sizeof(offsetStr), "%d", aqiConfig.offset);
        uint8_t dataBytes[4] = {0xFF, 0xFF, 0xFF, 0xFF};
        size_t len = strlen(offsetStr);
        if (len > 4) len = 4;
        memcpy(dataBytes, offsetStr, len);
        dwinSendVP(VP_AQI_CALIBRATION, dataBytes, 4);
      }
      Serial.printf("[MODBUS] AQI offset set to: %d\n", aqiConfig.offset);
      break;

    case HREG_ALARM_ACK:
      if (val == 1) {
        acknowledgeAllAlarms();
        // Reset the register value back to 0
        mb.Hreg(HREG_ALARM_ACK, 0);
      }
      break;
  }

  return val;
}

/* -------------------- MODBUS INITIALIZATION -------------------- */
void initModbus() {
  Serial.println("[MODBUS] Initializing...");

  // End previous serial if reinitializing
  modbusSerial.end();
  delay(10);

  // Start SoftwareSerial
  modbusSerial.begin(modbusBaudRate);

  // Initialize Modbus RTU
  mb.begin(&modbusSerial, MODBUS_DE_PIN);
  mb.slave(modbusSlaveId);

  // Add Holding Registers (R/W)
  for (uint16_t i = 0; i < HREG_COUNT; i++) {
    mb.addHreg(i, 0);
  }

  // Add Input Registers (R)
  for (uint16_t i = 0; i < IREG_COUNT; i++) {
    mb.addIreg(i, 0);
  }

  // Set initial values for Holding Registers
  mb.Hreg(HREG_TEMP_ENABLE, tempEnabled ? 0 : 1);
  mb.Hreg(HREG_HUMIDITY_ENABLE, humidityEnabled ? 0 : 1);
  mb.Hreg(HREG_PRESSURE_ENABLE, pressureEnabled ? 0 : 1);
  mb.Hreg(HREG_AQI_ENABLE, aqiEnabled ? 0 : 1);
  mb.Hreg(HREG_TEMP_OFFSET, (uint16_t)tempConfig.offset);
  mb.Hreg(HREG_HUMIDITY_OFFSET, (uint16_t)humidityConfig.offset);
  mb.Hreg(HREG_PRESSURE_OFFSET, (uint16_t)pressureConfig.offset);
  mb.Hreg(HREG_AQI_OFFSET, (uint16_t)aqiConfig.offset);
  mb.Hreg(HREG_ALARM_ACK, 0);

  // Set callback for Holding Register writes
  mb.onSetHreg(HREG_TEMP_ENABLE, modbusOnHregSet);
  mb.onSetHreg(HREG_HUMIDITY_ENABLE, modbusOnHregSet);
  mb.onSetHreg(HREG_PRESSURE_ENABLE, modbusOnHregSet);
  mb.onSetHreg(HREG_AQI_ENABLE, modbusOnHregSet);
  mb.onSetHreg(HREG_TEMP_OFFSET, modbusOnHregSet);
  mb.onSetHreg(HREG_HUMIDITY_OFFSET, modbusOnHregSet);
  mb.onSetHreg(HREG_PRESSURE_OFFSET, modbusOnHregSet);
  mb.onSetHreg(HREG_AQI_OFFSET, modbusOnHregSet);
  mb.onSetHreg(HREG_ALARM_ACK, modbusOnHregSet);

  modbusNeedsReinit = false;

  Serial.printf("[MODBUS] Initialized - Slave ID: %d, Baud: %lu\n", 
                modbusSlaveId, modbusBaudRate);
}

/* -------------------- MODBUS UPDATE INPUT REGISTERS -------------------- */
void updateModbusInputRegisters() {
  if (!modbusEnabled) return;

  // Temperature
  if (tempEnabled && tempConfig.dataValid) {
    mb.Ireg(IREG_TEMP_VALUE, (uint16_t)currentTempValue);
  } else {
    mb.Ireg(IREG_TEMP_VALUE, (uint16_t)MODBUS_DISABLED_VALUE);
  }

  // Humidity
  if (humidityEnabled && humidityConfig.dataValid) {
    mb.Ireg(IREG_HUMIDITY_VALUE, (uint16_t)currentHumidityValue);
  } else {
    mb.Ireg(IREG_HUMIDITY_VALUE, (uint16_t)MODBUS_DISABLED_VALUE);
  }

  // Pressure
  if (pressureEnabled && pressureConfig.dataValid) {
    mb.Ireg(IREG_PRESSURE_VALUE, (uint16_t)currentPressureValue);
  } else {
    mb.Ireg(IREG_PRESSURE_VALUE, (uint16_t)MODBUS_DISABLED_VALUE);
  }

  // AQI
  if (aqiEnabled && aqiConfig.dataValid) {
    mb.Ireg(IREG_AQI_VALUE, (uint16_t)currentAqiValue);
  } else {
    mb.Ireg(IREG_AQI_VALUE, (uint16_t)MODBUS_DISABLED_VALUE);
  }

  // Units (0=°F/mmH₂O, 1=°C/Pa)
  mb.Ireg(IREG_TEMP_UNIT, (tempUnit == UNIT_C) ? 1 : 0);
  mb.Ireg(IREG_PRESSURE_UNIT, (pressureUnit == UNIT_PA) ? 1 : 0);
}

/* -------------------- MODBUS TASK -------------------- */
void modbusTask(void *pvParameters) {
  while (1) {
    // Check if reinitialization needed (settings changed)
    if (modbusNeedsReinit && modbusEnabled) {
      initModbus();
    }

    // Process MODBUS requests only if enabled
    if (modbusEnabled) {
      mb.task();
      yield();
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/* -------------------- SENSOR READING & DATA REFRESH TASK -------------------- */
void dwinDataRefreshTask(void *pvParameters) {
  while (1) {
    // Default to NaN (invalid data)
    String temp_str = "NaN";
    String hum_str  = "NaN";
    String aqi_str  = "NaN";
    String pres_str = "NaN";  // No pressure sensor

    // Reset data valid flags
    tempConfig.dataValid = false;
    humidityConfig.dataValid = false;
    pressureConfig.dataValid = false;  // No sensor, always false
    aqiConfig.dataValid = false;

    float t = NAN, h = NAN;

    if (sht31_ok) {
      t = sht31.readTemperature();
      h = sht31.readHumidity();
      
      // Temperature
      if (!isnan(t)) {
        // Unit conversion first, then apply offset
        int temp_int = round(t);
        if (tempUnit == UNIT_F) {
          temp_int = round(t * 1.8f + 32.0f);
        }
        // Apply calibration offset after unit conversion
        temp_int = temp_int + tempConfig.offset;
        currentTempValue = temp_int;
        temp_str = String(temp_int);
        tempConfig.dataValid = true;
      }

      // Humidity
      if (!isnan(h)) {
        int hum_int = (int)round(h);
        hum_int = hum_int + humidityConfig.offset;
        currentHumidityValue = hum_int;
        hum_str = String(hum_int);
        humidityConfig.dataValid = true;
      }
    }

    // AQI (VOC Index) - only if temp/hum valid and SGP40 ok
    if (!isnan(t) && !isnan(h) && sgp_ok) {
      int32_t voc = sgp.measureVocIndex(t, h);
      // Apply calibration offset
      int aqi_int = voc + aqiConfig.offset;
      currentAqiValue = aqi_int;
      aqi_str = String(aqi_int);
      aqiConfig.dataValid = true;
    }

    // Pressure - No sensor integrated, always NaN
    // pressureConfig.dataValid remains false
    // pres_str remains "NaN"

    // Check alarms for each parameter (also resets acknowledged flag when back in limits)
    tempConfig.alarmActive = checkAlarmCondition(currentTempValue, &tempConfig);
    humidityConfig.alarmActive = checkAlarmCondition(currentHumidityValue, &humidityConfig);
    pressureConfig.alarmActive = checkAlarmCondition(currentPressureValue, &pressureConfig);
    aqiConfig.alarmActive = checkAlarmCondition(currentAqiValue, &aqiConfig);

    // Send data to active widgets
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

    // Update MODBUS input registers with latest sensor data
    updateModbusInputRegisters();
    
    // Sync holding registers (with flag to prevent callback)
    if (modbusEnabled) {
      modbusInternalWrite = true;  // Set flag
      
      mb.Hreg(HREG_TEMP_ENABLE, tempEnabled ? 0 : 1);
      mb.Hreg(HREG_HUMIDITY_ENABLE, humidityEnabled ? 0 : 1);
      mb.Hreg(HREG_PRESSURE_ENABLE, pressureEnabled ? 0 : 1);
      mb.Hreg(HREG_AQI_ENABLE, aqiEnabled ? 0 : 1);
      mb.Hreg(HREG_TEMP_OFFSET, (uint16_t)tempConfig.offset);
      mb.Hreg(HREG_HUMIDITY_OFFSET, (uint16_t)humidityConfig.offset);
      mb.Hreg(HREG_PRESSURE_OFFSET, (uint16_t)pressureConfig.offset);
      mb.Hreg(HREG_AQI_OFFSET, (uint16_t)aqiConfig.offset);
      
      modbusInternalWrite = false;  // Clear flag
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
    
    // Alarm Acknowledge Button
    case VP_ALARM_ACK:
      Serial.println("[ACK] Alarm acknowledge button pressed");
      acknowledgeAllAlarms();
      break;

    // Buzzer Sound Setting
    case VP_BUZZER_SETTING:
      buzzerEnabled = (value == 0);
      Serial.printf("[BUZZER] Alarm sound %s\n", buzzerEnabled ? "ON" : "OFF");
      break;

    // MODBUS Enable/Disable
    case VP_MODBUS_ENABLE:
      modbusEnabled = (value == 0);
      Serial.printf("[MODBUS] %s\n", modbusEnabled ? "ENABLED" : "DISABLED");
      break;
  }

  if (changed) {
    refreshDashboard();
  }
}

/* -------------------- MODBUS EEPROM FUNCTIONS -------------------- */
void loadModbusSettingsFromEEPROM() {
  preferences.begin("modbus", true);  // Read-only mode
  modbusSlaveId = preferences.getUChar("slaveId", MODBUS_DEFAULT_SLAVE_ID);
  modbusBaudRate = preferences.getULong("baudRate", MODBUS_DEFAULT_BAUD);
  preferences.end();

  // Validate slave ID (1-247)
  if (modbusSlaveId < 1 || modbusSlaveId > 247) {
    modbusSlaveId = MODBUS_DEFAULT_SLAVE_ID;
  }

  Serial.printf("[MODBUS] Loaded settings - Slave ID: %d, Baud Rate: %lu\n", 
                modbusSlaveId, modbusBaudRate);
}

void saveModbusSlaveIdToEEPROM(uint8_t slaveId) {
  if (slaveId < 1 || slaveId > 247) {
    Serial.println("[MODBUS] Invalid slave ID, not saving");
    return;
  }
  
  preferences.begin("modbus", false);  // Read-write mode
  preferences.putUChar("slaveId", slaveId);
  preferences.end();

  modbusSlaveId = slaveId;
  modbusNeedsReinit = true;
  Serial.printf("[MODBUS] Saved Slave ID: %d\n", modbusSlaveId);
}

void saveModbusBaudRateToEEPROM(uint32_t baudRate) {
  // Validate baud rate
  if (baudRate != 9600 && baudRate != 19200 && baudRate != 38400 && 
      baudRate != 57600 && baudRate != 115200) {
    Serial.printf("[MODBUS] Invalid baud rate: %lu, not saving\n", baudRate);
    return;
  }
  
  preferences.begin("modbus", false);  // Read-write mode
  preferences.putULong("baudRate", baudRate);
  preferences.end();

  modbusBaudRate = baudRate;
  modbusNeedsReinit = true;
  Serial.printf("[MODBUS] Saved Baud Rate: %lu\n", modbusBaudRate);
}

void handleStringVP(uint16_t vp, const char *text) {
  Serial.printf("STR  VP 0x%04X = \"%s\"\n", vp, text);

  // Handle Password VPs first
  switch (vp) {
    // -------- Settings Password Entry --------
    case VP_SETTINGS_PASSWORD_ENTRY:
      Serial.printf("  -> Settings password attempt: %s\n", text);
      if (validatePassword(text, settingsPassword)) {
        Serial.println("  -> Password CORRECT! Switching to Settings page.");
        dwinSwitchPage(PAGE_SETTINGS);
      } else {
        Serial.println("  -> Password INCORRECT!");
      }
      return;

    // -------- Settings Password Change --------
    case VP_SETTINGS_PASSWORD_CHANGE:
      Serial.printf("  -> Settings password change request: %s\n", text);
      if (isValidPasswordFormat(text)) {
        savePasswordToEEPROM(text);
        Serial.println("  -> New password saved successfully!");
      } else {
        Serial.println("  -> Invalid password format! Must be 6 digits.");
      }
      return;

    // -------- Advanced Settings Password Entry --------
    case VP_ADVANCED_PASSWORD_ENTRY:
      Serial.printf("  -> Advanced settings password attempt: %s\n", text);
      if (validatePassword(text, ADVANCED_SETTINGS_PASSWORD)) {
        Serial.println("  -> Password CORRECT! Switching to Advanced Settings page.");
        dwinSwitchPage(PAGE_ADVANCED_SETTINGS);
      } else {
        Serial.println("  -> Password INCORRECT!");
      }
      return;

    // -------- Date Input --------
    case VP_DATE_INPUT:
      Serial.printf("  -> Date input received: %s\n", text);
      if (parseDateInput(text)) {
        Serial.println("  -> Date updated successfully!");
      } else {
        Serial.println("  -> Failed to parse date!");
      }
      return;

    // -------- Time Input --------
    case VP_TIME_INPUT:
      Serial.printf("  -> Time input received: %s\n", text);
      if (parseTimeInput(text)) {
        Serial.println("  -> Time updated successfully!");
      } else {
        Serial.println("  -> Failed to parse time!");
      }
      return;
    
    // -------- MODBUS Slave ID --------
    case VP_MODBUS_SLAVE_ID:
      Serial.printf("  -> MODBUS Slave ID input: %s\n", text);
      if (isValidNumber(text)) {
        int slaveId = atoi(text);
        if (slaveId >= 1 && slaveId <= 247) {
          saveModbusSlaveIdToEEPROM((uint8_t)slaveId);
          Serial.printf("  -> Slave ID set to: %d\n", slaveId);
        } else {
          Serial.println("  -> Invalid Slave ID (must be 1-247)");
        }
      }
      return;

    // -------- MODBUS Baud Rate --------
    case VP_MODBUS_BAUD_RATE:
      Serial.printf("  -> MODBUS Baud Rate input: %s\n", text);
      if (isValidNumber(text)) {
        uint32_t baudRate = (uint32_t)atol(text);
        saveModbusBaudRateToEEPROM(baudRate);
        Serial.printf("  -> Baud Rate set to: %lu\n", baudRate);
      }
      return;
  }

  // Handle other string VPs (calibration & limits)
  bool validNum = isValidNumber(text);
  int16_t value = validNum ? parseAsciiToInt(text) : 0;

  switch (vp) {
    // -------- Calibration Offset VPs --------
    case VP_TEMP_CALIBRATION:
      tempConfig.offset = validNum ? value : 0;
      Serial.printf("  -> Temp calibration offset: %d (valid: %s)\n", 
                    tempConfig.offset, validNum ? "yes" : "no");
      break;
    case VP_HUMIDITY_CALIBRATION:
      humidityConfig.offset = validNum ? value : 0;
      Serial.printf("  -> Humidity calibration offset: %d (valid: %s)\n", 
                    humidityConfig.offset, validNum ? "yes" : "no");
      break;
    case VP_PRESSURE_CALIBRATION:
      pressureConfig.offset = validNum ? value : 0;
      Serial.printf("  -> Pressure calibration offset: %d (valid: %s)\n", 
                    pressureConfig.offset, validNum ? "yes" : "no");
      break;
    case VP_AQI_CALIBRATION:
      aqiConfig.offset = validNum ? value : 0;
      Serial.printf("  -> AQI calibration offset: %d (valid: %s)\n", 
                    aqiConfig.offset, validNum ? "yes" : "no");
      break;

    // -------- Upper Limit VPs --------
    case VP_TEMP_UPPER_LIMIT:
      tempConfig.upperLimitSet = validNum;
      tempConfig.upperLimit = validNum ? value : 0;
      Serial.printf("  -> Temp upper limit: %d (set: %s)\n", 
                    tempConfig.upperLimit, validNum ? "yes" : "no");
      break;
    case VP_HUMIDITY_UPPER_LIMIT:
      humidityConfig.upperLimitSet = validNum;
      humidityConfig.upperLimit = validNum ? value : 0;
      Serial.printf("  -> Humidity upper limit: %d (set: %s)\n", 
                    humidityConfig.upperLimit, validNum ? "yes" : "no");
      break;
    case VP_PRESSURE_UPPER_LIMIT:
      pressureConfig.upperLimitSet = validNum;
      pressureConfig.upperLimit = validNum ? value : 0;
      Serial.printf("  -> Pressure upper limit: %d (set: %s)\n", 
                    pressureConfig.upperLimit, validNum ? "yes" : "no");
      break;
    case VP_AQI_UPPER_LIMIT:
      aqiConfig.upperLimitSet = validNum;
      aqiConfig.upperLimit = validNum ? value : 0;
      Serial.printf("  -> AQI upper limit: %d (set: %s)\n", 
                    aqiConfig.upperLimit, validNum ? "yes" : "no");
      break;

    // -------- Lower Limit VPs --------
    case VP_TEMP_LOWER_LIMIT:
      tempConfig.lowerLimitSet = validNum;
      tempConfig.lowerLimit = validNum ? value : 0;
      Serial.printf("  -> Temp lower limit: %d (set: %s)\n", 
                    tempConfig.lowerLimit, validNum ? "yes" : "no");
      break;
    case VP_HUMIDITY_LOWER_LIMIT:
      humidityConfig.lowerLimitSet = validNum;
      humidityConfig.lowerLimit = validNum ? value : 0;
      Serial.printf("  -> Humidity lower limit: %d (set: %s)\n", 
                    humidityConfig.lowerLimit, validNum ? "yes" : "no");
      break;
    case VP_PRESSURE_LOWER_LIMIT:
      pressureConfig.lowerLimitSet = validNum;
      pressureConfig.lowerLimit = validNum ? value : 0;
      Serial.printf("  -> Pressure lower limit: %d (set: %s)\n", 
                    pressureConfig.lowerLimit, validNum ? "yes" : "no");
      break;
    case VP_AQI_LOWER_LIMIT:
      aqiConfig.lowerLimitSet = validNum;
      aqiConfig.lowerLimit = validNum ? value : 0;
      Serial.printf("  -> AQI lower limit: %d (set: %s)\n", 
                    aqiConfig.lowerLimit, validNum ? "yes" : "no");
      break;

    // -------- Data Display VPs (logging only) --------
    case 0x5012:
    case 0x5022:
    case 0x5032:
    case 0x5042:
      break;

    default:
      Serial.printf("  -> Unhandled string VP\n");
      break;
  }
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
    while (i < maxBytes && i < sizeof(text) - 1 && data[i] != 0xFF && data[i] != 0x00) {
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

/* -------------------- READ CALIBRATION & LIMITS ON BOOT -------------------- */
void readCalibrationAndLimitsFromDisplay() {
  // Read Calibration Offsets (2 words = 4 bytes each)
  dwinReadVP(VP_TEMP_CALIBRATION, 2);
  vTaskDelay(pdMS_TO_TICKS(50));
  dwinReadVP(VP_HUMIDITY_CALIBRATION, 2);
  vTaskDelay(pdMS_TO_TICKS(50));
  dwinReadVP(VP_PRESSURE_CALIBRATION, 2);
  vTaskDelay(pdMS_TO_TICKS(50));
  dwinReadVP(VP_AQI_CALIBRATION, 2);
  vTaskDelay(pdMS_TO_TICKS(50));

  // Read Upper Limits (2 words = 4 bytes each)
  dwinReadVP(VP_TEMP_UPPER_LIMIT, 2);
  vTaskDelay(pdMS_TO_TICKS(50));
  dwinReadVP(VP_HUMIDITY_UPPER_LIMIT, 2);
  vTaskDelay(pdMS_TO_TICKS(50));
  dwinReadVP(VP_PRESSURE_UPPER_LIMIT, 2);
  vTaskDelay(pdMS_TO_TICKS(50));
  dwinReadVP(VP_AQI_UPPER_LIMIT, 2);
  vTaskDelay(pdMS_TO_TICKS(50));

  // Read Lower Limits (2 words = 4 bytes each)
  dwinReadVP(VP_TEMP_LOWER_LIMIT, 2);
  vTaskDelay(pdMS_TO_TICKS(50));
  dwinReadVP(VP_HUMIDITY_LOWER_LIMIT, 2);
  vTaskDelay(pdMS_TO_TICKS(50));
  dwinReadVP(VP_PRESSURE_LOWER_LIMIT, 2);
  vTaskDelay(pdMS_TO_TICKS(50));
  dwinReadVP(VP_AQI_LOWER_LIMIT, 2);
  vTaskDelay(pdMS_TO_TICKS(50));
}

/* -------------------- ARDUINO SETUP -------------------- */
void setup() {
  Serial.begin(115200);
  delay(3000);  // Wait for display to initialize
  Serial.println("===========================================");
  Serial.println("ESP32 + FreeRTOS + DWIN + Sensors");
  Serial.println("===========================================");

  // Load password from EEPROM
  loadPasswordFromEEPROM();

  Wire.begin(21, 22);  // SDA, SCL

  sht31_ok = sht31.begin(0x44);
  if (!sht31_ok) Serial.println("[ERROR] SHT31 not found");
  else Serial.println("[OK] SHT31 initialized");

  sgp_ok = sgp.begin();
  if (!sgp_ok) Serial.println("[ERROR] SGP40 not found");
  else Serial.println("[OK] SGP40 initialized");

  DWIN_UART.begin(DWIN_BAUD, SERIAL_8N1, DWIN_RX_PIN, DWIN_TX_PIN);
  Serial.println("[OK] DWIN UART initialized");

  dwinUartMutex = xSemaphoreCreateMutex();
  if (dwinUartMutex == NULL) {
    Serial.println("[FATAL] Failed to create DWIN UART mutex");
    while (1);
  }
  Serial.println("[OK] DWIN UART mutex created");

  rtcMutex = xSemaphoreCreateMutex();
  if (rtcMutex == NULL) {
    Serial.println("[FATAL] Failed to create RTC mutex");
    while (1);
  }
  Serial.println("[OK] RTC mutex created");

  // Create FreeRTOS tasks
  xTaskCreate(dwinRxTask, "DWIN_RX", 4096, NULL, 2, NULL);
  xTaskCreate(dwinDataRefreshTask, "DWIN_REFRESH", 4096, NULL, 2, NULL);
  xTaskCreate(alarmBlinkTask, "ALARM_BLINK", 2048, NULL, 1, NULL);
  xTaskCreate(rtcTask, "RTC_TASK", 2048, NULL, 1, NULL);
  Serial.println("[OK] FreeRTOS tasks created");

  // Initial state sync - Settings VPs
  Serial.println("Reading settings VPs...");
  dwinReadVP(0x5110, 1);  // Temp unit
  vTaskDelay(pdMS_TO_TICKS(50));
  dwinReadVP(0x5111, 1);  // Temp enable
  vTaskDelay(pdMS_TO_TICKS(50));
  dwinReadVP(0x5120, 1);  // Pressure unit
  vTaskDelay(pdMS_TO_TICKS(50));
  dwinReadVP(0x5121, 1);  // Pressure enable
  vTaskDelay(pdMS_TO_TICKS(50));
  dwinReadVP(0x5131, 1);  // Humidity enable
  vTaskDelay(pdMS_TO_TICKS(50));
  dwinReadVP(0x5141, 1);  // AQI enable
  vTaskDelay(pdMS_TO_TICKS(50));

  // Read calibration and limits from display
  Serial.println("Reading calibration & limits VPs...");
  readCalibrationAndLimitsFromDisplay();

  // Send initial date/time to display
  Serial.println("Initializing RTC...");
  Serial.printf("[RTC] Default: %02d/%02d/%04d %02d:%02d:%02d\n", 
                rtcDay, rtcMonth, 2000 + rtcYear, 
                rtcHour, rtcMinute, rtcSecond);
  dwinUpdateRTC();

  // Load MODBUS settings from EEPROM
  loadModbusSettingsFromEEPROM();

  // Initialize MODBUS
  initModbus();

  // Create MODBUS task
  xTaskCreate(modbusTask, "MODBUS_TASK", 4096, NULL, 1, NULL);
  Serial.println("[OK] MODBUS task created");

  // Log default buzzer setting
  Serial.printf("[BUZZER] Default: Alarm sound %s\n", buzzerEnabled ? "ON" : "OFF");

  Serial.println("===========================================");
  Serial.println("Setup complete! System running.");
  Serial.println("===========================================");
}

/* -------------------- ARDUINO LOOP -------------------- */
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}