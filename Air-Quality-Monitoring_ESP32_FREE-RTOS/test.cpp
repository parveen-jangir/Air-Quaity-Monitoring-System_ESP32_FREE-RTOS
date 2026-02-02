/*
 * Arduino Uno Modbus RTU Master - Complete System Test
 * For testing ESP32 Air Quality Monitor MODBUS Slave
 * 
 * Hardware: MAX485 Module
 * Arduino D10 → RO (RX)
 * Arduino D11 → DI (TX)
 * Arduino D2  → DE
 * Arduino D3  → RE
 */

#include <ModbusMaster.h>
#include <SoftwareSerial.h>

#define MAX485_DE   2
#define MAX485_RE   3
#define SLAVE_ID    1

// Holding Register Addresses (R/W)
#define HREG_TEMP_ENABLE      0
#define HREG_HUMIDITY_ENABLE  1
#define HREG_PRESSURE_ENABLE  2
#define HREG_AQI_ENABLE       3
#define HREG_TEMP_OFFSET      4
#define HREG_HUMIDITY_OFFSET  5
#define HREG_PRESSURE_OFFSET  6
#define HREG_AQI_OFFSET       7
#define HREG_ALARM_ACK        8

// Input Register Addresses (R)
#define IREG_TEMP_VALUE       0
#define IREG_HUMIDITY_VALUE   1
#define IREG_PRESSURE_VALUE   2
#define IREG_AQI_VALUE        3
#define IREG_TEMP_UNIT        4
#define IREG_PRESSURE_UNIT    5

ModbusMaster node;
SoftwareSerial mSerial(10, 11);

void preTransmission() {
  digitalWrite(MAX485_RE, HIGH);
  digitalWrite(MAX485_DE, HIGH);
}

void postTransmission() {
  digitalWrite(MAX485_RE, LOW);
  digitalWrite(MAX485_DE, LOW);
}

void setup() {
  pinMode(MAX485_RE, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  
  digitalWrite(MAX485_RE, LOW);
  digitalWrite(MAX485_DE, LOW);
  
  mSerial.begin(9600);
  Serial.begin(9600);
  
  node.begin(SLAVE_ID, mSerial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  
  delay(1000);
  
  Serial.println(F("=== MODBUS Master ==="));
  Serial.println(F("r=read i=input h=hold"));
  Serial.println(F("1=tempON 2=tempOFF"));
  Serial.println(F("3=humON 4=humOFF"));
  Serial.println(F("5=aqiON 6=aqiOFF"));
  Serial.println(F("7=off+5 8=off-3 9=off0"));
  Serial.println(F("a=ack c=cont"));
}

// Read Input Registers (sensor values)
void readInput() {
  Serial.println(F("\n-- INPUT --"));
  
  uint8_t result = node.readInputRegisters(0, 6);
  
  if (result == node.ku8MBSuccess) {
    int16_t temp = (int16_t)node.getResponseBuffer(0);
    int16_t hum = (int16_t)node.getResponseBuffer(1);
    int16_t pres = (int16_t)node.getResponseBuffer(2);
    int16_t aqi = (int16_t)node.getResponseBuffer(3);
    uint16_t tUnit = node.getResponseBuffer(4);
    uint16_t pUnit = node.getResponseBuffer(5);
    
    Serial.print(F("Temp:"));
    if (temp == -9999) Serial.println(F("OFF"));
    else { Serial.print(temp); Serial.println(tUnit ? F("C") : F("F")); }
    
    Serial.print(F("Hum:"));
    if (hum == -9999) Serial.println(F("OFF"));
    else { Serial.print(hum); Serial.println(F("%")); }
    
    Serial.print(F("Pres:"));
    if (pres == -9999) Serial.println(F("OFF"));
    else { Serial.print(pres); Serial.println(pUnit ? F("Pa") : F("mm")); }
    
    Serial.print(F("AQI:"));
    if (aqi == -9999) Serial.println(F("OFF"));
    else Serial.println(aqi);
  } else {
    Serial.print(F("Err:"));
    Serial.println(result);
  }
}

// Read Holding Registers (settings)
void readHolding() {
  Serial.println(F("\n-- HOLDING --"));
  
  uint8_t result = node.readHoldingRegisters(0, 9);
  
  if (result == node.ku8MBSuccess) {
    Serial.print(F("TempEn:"));
    Serial.println(node.getResponseBuffer(0) == 0 ? F("ON") : F("OFF"));
    
    Serial.print(F("HumEn:"));
    Serial.println(node.getResponseBuffer(1) == 0 ? F("ON") : F("OFF"));
    
    Serial.print(F("PresEn:"));
    Serial.println(node.getResponseBuffer(2) == 0 ? F("ON") : F("OFF"));
    
    Serial.print(F("AqiEn:"));
    Serial.println(node.getResponseBuffer(3) == 0 ? F("ON") : F("OFF"));
    
    Serial.print(F("TempOff:"));
    Serial.println((int16_t)node.getResponseBuffer(4));
    
    Serial.print(F("HumOff:"));
    Serial.println((int16_t)node.getResponseBuffer(5));
    
    Serial.print(F("PresOff:"));
    Serial.println((int16_t)node.getResponseBuffer(6));
    
    Serial.print(F("AqiOff:"));
    Serial.println((int16_t)node.getResponseBuffer(7));
  } else {
    Serial.print(F("Err:"));
    Serial.println(result);
  }
}

// Write single register
bool writeReg(uint16_t reg, uint16_t val) {
  Serial.print(F("W["));
  Serial.print(reg);
  Serial.print(F("]="));
  Serial.print((int16_t)val);
  
  uint8_t result = node.writeSingleRegister(reg, val);
  
  if (result == node.ku8MBSuccess) {
    Serial.println(F(" OK"));
    return true;
  } else {
    Serial.print(F(" Err:"));
    Serial.println(result);
    return false;
  }
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) Serial.read();
    
    switch (c) {
      case 'r':  // Read all
        readInput();
        readHolding();
        break;
        
      case 'i':  // Read input only
        readInput();
        break;
        
      case 'h':  // Read holding only
        readHolding();
        break;
        
      case '1':  // Temp ON
        writeReg(HREG_TEMP_ENABLE, 0);
        delay(500);
        readInput();
        break;
        
      case '2':  // Temp OFF
        writeReg(HREG_TEMP_ENABLE, 1);
        delay(500);
        readInput();
        break;
        
      case '3':  // Humidity ON
        writeReg(HREG_HUMIDITY_ENABLE, 0);
        delay(500);
        readInput();
        break;
        
      case '4':  // Humidity OFF
        writeReg(HREG_HUMIDITY_ENABLE, 1);
        delay(500);
        readInput();
        break;
        
      case '5':  // AQI ON
        writeReg(HREG_AQI_ENABLE, 0);
        delay(500);
        readInput();
        break;
        
      case '6':  // AQI OFF
        writeReg(HREG_AQI_ENABLE, 1);
        delay(500);
        readInput();
        break;
        
      case '7':  // Temp offset +5
        writeReg(HREG_TEMP_OFFSET, 5);
        delay(500);
        readHolding();
        break;
        
      case '8':  // Temp offset -3
        writeReg(HREG_TEMP_OFFSET, (uint16_t)(-3));
        delay(500);
        readHolding();
        break;
        
      case '9':  // Temp offset 0
        writeReg(HREG_TEMP_OFFSET, 0);
        delay(500);
        readHolding();
        break;
        
      case 'a':  // Alarm ACK
        writeReg(HREG_ALARM_ACK, 1);
        break;
        
      case 'c':  // Continuous read
        Serial.println(F("\n-- CONTINUOUS (5x) --"));
        for (uint8_t i = 0; i < 5; i++) {
          Serial.print(F("\n["));
          Serial.print(i + 1);
          Serial.println(F("]"));
          readInput();
          delay(3000);
        }
        Serial.println(F("-- DONE --"));
        break;
        
      case 't':  // Full test sequence
        Serial.println(F("\n=== FULL TEST ==="));
        
        Serial.println(F("\n[1] Read all"));
        readInput();
        readHolding();
        delay(1000);
        
        Serial.println(F("\n[2] Disable temp"));
        writeReg(HREG_TEMP_ENABLE, 1);
        delay(500);
        readInput();
        delay(1000);
        
        Serial.println(F("\n[3] Enable temp"));
        writeReg(HREG_TEMP_ENABLE, 0);
        delay(500);
        readInput();
        delay(1000);
        
        Serial.println(F("\n[4] Set offset +10"));
        writeReg(HREG_TEMP_OFFSET, 10);
        delay(500);
        readInput();
        readHolding();
        delay(1000);
        
        Serial.println(F("\n[5] Set offset -5"));
        writeReg(HREG_TEMP_OFFSET, (uint16_t)(-5));
        delay(500);
        readInput();
        readHolding();
        delay(1000);
        
        Serial.println(F("\n[6] Reset offset"));
        writeReg(HREG_TEMP_OFFSET, 0);
        delay(500);
        readHolding();
        delay(1000);
        
        Serial.println(F("\n[7] Alarm ACK"));
        writeReg(HREG_ALARM_ACK, 1);
        delay(500);
        
        Serial.println(F("\n=== TEST DONE ==="));
        break;
        
      default:
        Serial.println(F("\nr=read i=in h=hold"));
        Serial.println(F("1/2=temp 3/4=hum 5/6=aqi"));
        Serial.println(F("7/8/9=offset a=ack"));
        Serial.println(F("c=cont t=fulltest"));
        break;
    }
  }
}

/*
Key	Function
r	Read all (Input + Holding)
i	Read Input registers only
h	Read Holding registers only
1	Temperature ON
2	Temperature OFF
3	Humidity ON
4	Humidity OFF
5	AQI ON
6	AQI OFF
7	Temp offset = +5
8	Temp offset = -3
9	Temp offset = 0
a	Alarm Acknowledge
c	Continuous read (5 times)
t	Full test sequence
*/