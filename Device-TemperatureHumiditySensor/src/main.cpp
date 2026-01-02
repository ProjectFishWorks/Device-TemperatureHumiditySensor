#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <NodeControllerCore.h>

// ---------------------------------------------------------put global variables here:---------------------------------------------------------

// #define debuging // Comment out to remove debug messages

#define messageGap 5000       // 5 seconds
#define sendMessageDelay 1000 // 1/10 second

// USE int FOR I2C PIN DEFINITIONS
int I2C_SDA = 3;
int I2C_SCL = 2;

// Define the SHT30-D sensor object
Adafruit_SHT31 sht30D = Adafruit_SHT31();

// Data wire is conntec to the Arduino digital pin 4
#define ONE_WIRE_BUS 4
#define TEMPERATURE_PRECISION 9

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature WaterTempSensors(&oneWire);

// Node controller core object
NodeControllerCore core;

// ------------------------------  Mesages and their case statement variables  ------------------------------
#define NODE_ID 0xA2 // Node ID in decimal is 162

#define CANOPY_TEMP_MESSAGE_ID 2560       // ID in hex is 0x1000
#define CANOPY_TEMP_ALARM_MESSAGE_ID 2561 // ID in hex is 0x1001
bool canopyTempAlarmOnOff;
#define CANOPY_TEMP_ALARM_LOW_MESSAGE_ID 2562 // ID in hex is 0x1002
int canopyTempAlarmLow = 0;
#define CANOPY_TEMP_ALARM_HIGH_MESSAGE_ID 2563 // ID in hex is 0x1003
int canopyTempAlarmHigh = 100;

#define CANOPY_HUMIDITY_MESSAGE_ID 2564       // ID in hex is 0x1004
#define CANOPY_HUMIDITY_ALARM_MESSAGE_ID 2565 // ID in hex is 0x1005
bool canopyHumidityAlarmOnOff;
#define CANOPY_HUMIDITY_ALARM_LOW_MESSAGE_ID 2566 // ID in hex is 0x1006
int canopyHumidityAlarmLow = 0;
#define CANOPY_HUMIDITY_ALARM_HIGH_MESSAGE_ID 2567 // ID in hex is 0x1007
int canopyHumidityAlarmHigh = 100;

#define TANK_TEMP_MESSAGE_ID 2568       // ID in hex is 0x1008
#define TANK_TEMP_ALARM_MESSAGE_ID 2569 // ID in hex is 0x1009
bool tankTempAlarmOnOff;
#define TANK_TEMP_ALARM_LOW_MESSAGE_ID 2570 // ID in hex is 0x100A
int tankTempAlarmLow = 0;
#define TANK_TEMP_ALARM_HIGH_MESSAGE_ID 2571 // ID in hex is 0x100B
int tankTempAlarmHigh = 100;

#define SUMP_TEMP_MESSAGE_ID 2572       // ID in hex is 0x100C
#define SUMP_TEMP_ALARM_MESSAGE_ID 2573 // ID in hex is 0x100D
bool sumpTempAlarmOnOff;
#define SUMP_TEMP_ALARM_LOW_MESSAGE_ID 2574 // ID in hex is 0x100E
int sumpTempAlarmLow = 0;
#define SUMP_TEMP_ALARM_HIGH_MESSAGE_ID 2575 // ID in hex is 0x100F
int sumpTempAlarmHigh = 100;

#define ALARM_MESSAGE_ID 901 // ID in hex is 0x385

uint8_t hasSentCanTempAlarm = 0;
uint8_t hasSentNoCanTempAlarm = 0;
uint8_t hasSentCanHumAlarm = 0;
uint8_t hasSentNoCanHumAlarm = 0;
uint8_t hasSentTankTempAlarm = 0;
uint8_t hasSentNoTankTempAlarm = 0;
uint8_t hasSentSumpTempAlarm = 0;
uint8_t hasSentNoSumpTempAlarm = 0;
uint64_t errorData1 = 1;
uint64_t errorData0 = 0;

// ------------------------------  Variables for the temperature and humidity values to be stored in  ------------------------------

float canopyTemp;
float CanopyTemp;
float canopyHum;
float CanopyHum;
float tankTemp;
float TankTemp;
float sumpTemp;
float SumpTemp;
int tankThermometer = 0;
int sumpThermometer = 1;

// ---------------------------------------------------------put function declarations here:--------------------------------------------------------

// What to do with received messages from the CAN bus
void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data);

// Task to send temperature and humidity messages
void SendTempHumMessage(void *parameters);

//------------------------------------------------------- put your setup code here, to run once:------------------------------------------------------
void setup()
{
  // Create the node controller core object
  core = NodeControllerCore();
  Serial.println("test");
  // Initialize the node controller core object
  if (core.Init(receive_message, NODE_ID))
  {
    Serial.println("Driver device initialized");
  }
  else
  {
    Serial.println("Failed to initialize driver");
  }

  // Initialize the I2C communication
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize the DS18B20 temperature sensors
  WaterTempSensors.begin();

  Serial.begin(115200);

  // Initialize the SHT30-D object and pass it the I2C address
  sht30D.begin(0x44);

  xTaskCreate(
      SendTempHumMessage,             /* Task function. */
      "SendTemperatureAndHumidities", /* name of task. */
      10000,                          /* Stack size of task */
      NULL,                           /* parameter of the task */
      1,                              /* priority of the task */
      NULL);                          /* Task handle to keep track of created task */
                                      /* pin task to core 0 */

  // retreive canopy temperature and humidity
  canopyTemp = sht30D.readTemperature();
  canopyHum = sht30D.readHumidity();

  // Call WaterTempSensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  WaterTempSensors.requestTemperatures();
  tankTemp = WaterTempSensors.getTempCByIndex(tankThermometer);
  sumpTemp = WaterTempSensors.getTempCByIndex(sumpThermometer);
  delay(messageGap);

#ifdef debuging
  Serial.println("Canopy Temperture = " + String(canopyTemp));
  Serial.println("Canopy Humidity = " + String(canopyHum));
  Serial.println("Tank Temperture = " + String(tankTemp));
  Serial.println("Sump Temperture = " + String(sumpTemp));
  Serial.println("End of setup");
  Serial.println("");
#endif
}

//---------------------------------------------------- put your main code here, to run repeatedly:----------------------------------------------------
void loop()
{
}

//--------------------------------------------------------- put function definitions here: ---------------------------------------------------------

// Callback function for received messages from the CAN bus
void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data)
{
#ifdef debuging
  Serial.println("Message received callback");
  Serial.println("Node ID: " + String(nodeID, HEX));
  Serial.println("Message ID: " + String(messageID, HEX));
  Serial.println("Data: " + String(data));
#endif

  // Check if the message is for this node
  if (nodeID == NODE_ID)
  {
#ifdef displayMSG
    Serial.println("Message received to self");
#endif

// Handle the message based on its ID
    switch (messageID)
    {
    case CANOPY_TEMP_ALARM_MESSAGE_ID:
      canopyTempAlarmOnOff = data;
#ifdef debuging
      Serial.println("Canopy Temperture alarm is " + String(data));
      Serial.println("");
      delay(messageGap);
#endif
      break;

    case CANOPY_TEMP_ALARM_LOW_MESSAGE_ID:
      canopyTempAlarmLow = data;
      hasSentCanTempAlarm = 0;
      hasSentNoCanTempAlarm = 0;
#ifdef debuging
      Serial.println("Canopy Temperture alarm low is " + String(data));
      Serial.println("");
      delay(messageGap);
#endif
      break;

    case CANOPY_TEMP_ALARM_HIGH_MESSAGE_ID:
      canopyTempAlarmHigh = data;
      hasSentCanTempAlarm = 0;
      hasSentNoCanTempAlarm = 0;
#ifdef debuging
      Serial.println("Canopy Temperture alarm high is " + String(data));
      Serial.println("");
      delay(messageGap);
#endif
      break;

    case CANOPY_HUMIDITY_ALARM_MESSAGE_ID:
      canopyHumidityAlarmOnOff = data;
#ifdef debuging
      Serial.println("Canopy Humidity alarm is " + String(data));
      Serial.println("");
      delay(messageGap);
#endif
      break;

    case CANOPY_HUMIDITY_ALARM_LOW_MESSAGE_ID:
      canopyHumidityAlarmLow = data;
      hasSentCanHumAlarm = 0;
      hasSentNoCanHumAlarm = 0;
#ifdef debuging
      Serial.println("Canopy Humidity alarm low is " + String(data));
      Serial.println("");
      delay(messageGap);
#endif
      break;

    case CANOPY_HUMIDITY_ALARM_HIGH_MESSAGE_ID:
      canopyHumidityAlarmHigh = data;
      hasSentCanHumAlarm = 0;
      hasSentNoCanHumAlarm = 0;
#ifdef debuging
      Serial.println("Canopy Humidity alarm high is " + String(data));
      Serial.println("");
      delay(messageGap);
#endif
      break;

    case TANK_TEMP_ALARM_MESSAGE_ID:
      tankTempAlarmOnOff = data;
#ifdef debuging
      Serial.println("Tank Temperture alarm is " + String(data));
      Serial.println("");
      delay(messageGap);
#endif
      break;

    case TANK_TEMP_ALARM_LOW_MESSAGE_ID:
      tankTempAlarmLow = data;
      hasSentTankTempAlarm = 0;
      hasSentNoTankTempAlarm = 0;
#ifdef debuging
      Serial.println("The low Temperture for the Tank alarm low is " + String(data));
      Serial.println("");
      delay(messageGap);
#endif
      break;

    case TANK_TEMP_ALARM_HIGH_MESSAGE_ID:
      tankTempAlarmHigh = data;
      hasSentTankTempAlarm = 0;
      hasSentNoTankTempAlarm = 0;
#ifdef debuging
      Serial.println("The high Temperture for the Tank alarm is = " + String(data));
      Serial.println("");
      delay(messageGap);
#endif
      break;

    case SUMP_TEMP_ALARM_MESSAGE_ID:
      sumpTempAlarmOnOff = data;
#ifdef debuging
      Serial.println("Sump Temperture alarm is " + String(data));
      Serial.println("");
      delay(messageGap);
#endif
      break;

    case SUMP_TEMP_ALARM_LOW_MESSAGE_ID:
      sumpTempAlarmLow = data;
      hasSentSumpTempAlarm = 0;
      hasSentNoSumpTempAlarm = 0;
#ifdef debuging
      Serial.println("The low Temperture for the sump alarm = " + String(data));
      Serial.println("");
      delay(messageGap);
#endif
      break;

    case SUMP_TEMP_ALARM_HIGH_MESSAGE_ID:
      sumpTempAlarmHigh = data;
      hasSentSumpTempAlarm = 0;
      hasSentNoSumpTempAlarm = 0;
#ifdef debuging
      Serial.println("The high Temperture for the Sump alarm = " + String(data));
      Serial.println("");
      delay(messageGap);
#endif
      break;

    default:
      break;
    }
  }
}

// Task to send temperature and humidity messages
void SendTempHumMessage(void *parameters)
{
  while (1)
  {
    canopyTemp = sht30D.readTemperature();
    canopyHum = sht30D.readHumidity();

    WaterTempSensors.requestTemperatures();
    tankTemp = WaterTempSensors.getTempCByIndex(tankThermometer);
    sumpTemp = WaterTempSensors.getTempCByIndex(sumpThermometer);
    core.sendMessage(SUMP_TEMP_MESSAGE_ID, &SumpTemp); // Send the sump temperature

#ifdef debuging
    Serial.println("\nRunning SendTempHumMessage Task\n");
    Serial.println("Canopy Temperture = " + String(canopyTemp));
    Serial.println("Canopy Humidity = " + String(canopyHum));
    Serial.println("Tank Temperture = " + String(tankTemp));
    Serial.println("Sump Temperture = " + String(sumpTemp));
    Serial.println("");
    delay(messageGap);
#endif

    delay(messageGap);

    CanopyTemp = canopyTemp;
    CanopyHum = canopyHum;
    if (CanopyTemp > -250)
    {
      core.sendMessage(CANOPY_TEMP_MESSAGE_ID, &CanopyTemp); // Send the Canopy temperature
    }
    delay(sendMessageDelay);
    core.sendMessage(CANOPY_HUMIDITY_MESSAGE_ID, &CanopyHum); // Send the Canopy humidity
    delay(sendMessageDelay);
    TankTemp = tankTemp;
    SumpTemp = sumpTemp;
    core.sendMessage(TANK_TEMP_MESSAGE_ID, &TankTemp); // Send the tank temperature
    delay(sendMessageDelay);
    core.sendMessage(SUMP_TEMP_MESSAGE_ID, &SumpTemp); // Send the sump temperature
    delay(sendMessageDelay);

    if (canopyTempAlarmOnOff == 1)
    {
      if (CanopyTemp >= canopyTempAlarmHigh || CanopyTemp <= canopyTempAlarmLow)
      {
#ifdef debuging
        Serial.println("\n------------Canopy Temp Message triggered");
        Serial.println("Has sent alarm is = " + String(hasSentCanTempAlarm));
        Serial.println("");
#endif

        if (hasSentCanTempAlarm == 0)
        {
          core.sendMessage(ALARM_MESSAGE_ID, &errorData1);
          hasSentCanTempAlarm = 1;
          hasSentNoCanTempAlarm = 0;

#ifdef debuging
          Serial.println("-----------Not Can Temp Alarm  = " + String(hasSentCanTempAlarm));
          Serial.println("");
#endif
        }
      }
      else
      {
        if (hasSentNoCanTempAlarm == 0)
        {
          core.sendMessage(ALARM_MESSAGE_ID, &errorData0);
          hasSentCanTempAlarm = 0;
          hasSentNoCanTempAlarm = 1;

#ifdef debuging
          Serial.println("Canopy Temperture no alarm");
          Serial.println("Canopy Temperture = " + String(CanopyTemp));
          Serial.println("");
          delay(messageGap);
#endif
        }
      }
    }

#ifdef debuging
    Serial.println("Canopy Temperture canopyHumidityAlarmOnOff= " + String(canopyHumidityAlarmOnOff));
    Serial.println("");
    delay(messageGap);
#endif

    if (canopyHumidityAlarmOnOff == 1)
    {
      if (CanopyHum >= canopyHumidityAlarmHigh || CanopyHum <= canopyHumidityAlarmLow)
      {

#ifdef debuging
        Serial.println("------------Send Canopy Humidity alarm triggered");
        Serial.println("Has sent Can Hum alarm is = " + String(hasSentCanHumAlarm));
        Serial.println("");
        delay(messageGap);
#endif

        if (hasSentCanHumAlarm == 0)
        {
          core.sendMessage(ALARM_MESSAGE_ID, &errorData1);
          delay(sendMessageDelay);
          hasSentCanHumAlarm = 1;
          hasSentNoCanHumAlarm = 0;

#ifdef debuging
          Serial.println("-----------Not has sent Can Hum Alarm  = " + String(hasSentCanHumAlarm));
          Serial.println("");
          delay(messageGap);
#endif
        }
      }
      else
      {
        if (hasSentNoCanHumAlarm == 0)
        {
          core.sendMessage(ALARM_MESSAGE_ID, &errorData0);
          delay(sendMessageDelay);
          hasSentCanHumAlarm = 0;
          hasSentNoCanHumAlarm = 1;

#ifdef debuging
          Serial.println("Canopy Humidity no alarm");
          Serial.println("Canopy Humidity = " + String(CanopyHum));
          Serial.println("");
          delay(messageGap);
#endif
        }
      }
    }

    if (tankTempAlarmOnOff == 1)
    {
      if (TankTemp >= tankTempAlarmHigh || TankTemp <= tankTempAlarmLow)
      {

#ifdef debuging
        Serial.println("------------Send Tank Temp alarm triggered");
        Serial.println("Has sent Tank Temp Alarm is = " + String(hasSentTankTempAlarm));
        Serial.println("");
        delay(messageGap);
#endif

        if (hasSentTankTempAlarm == 0)
        {
          core.sendMessage(ALARM_MESSAGE_ID, &errorData1);
          delay(sendMessageDelay);
          hasSentTankTempAlarm = 1;
          hasSentNoTankTempAlarm = 0;

#ifdef debuging
          Serial.println("----------- Has Not sent Tank Temp Alarm  = " + String(hasSentTankTempAlarm));
          Serial.println("");
          delay(messageGap);
#endif
        }
      }
      else
      {
        if (hasSentNoTankTempAlarm == 1)
        {
          core.sendMessage(ALARM_MESSAGE_ID, &errorData0);
          delay(sendMessageDelay);
          hasSentTankTempAlarm = 0;
          hasSentNoTankTempAlarm = 1;

#ifdef debuging
          Serial.println("Tank Temperture no alarm");
          Serial.println("Tank Temperture = " + String(TankTemp));
          Serial.println("");
          delay(messageGap);
#endif
        }
      }
    }

    if (sumpTempAlarmOnOff == 1)
    {

#ifdef debuging
      Serial.println("If Sump Temperture = " + String(SumpTemp));
      Serial.println("");
      delay(messageGap);
#endif

      if (SumpTemp >= sumpTempAlarmHigh || SumpTemp <= sumpTempAlarmLow)
      {
#ifdef debuging
        Serial.println("------------Send Sump Temp alarm triggered");
        Serial.println("Has sent Sump Temp alarm is = " + String(hasSentSumpTempAlarm));
        Serial.println("");
        delay(messageGap);
#endif
        if (hasSentSumpTempAlarm == 0)
        {
          core.sendMessage(ALARM_MESSAGE_ID, &errorData1);
          delay(sendMessageDelay);
          hasSentSumpTempAlarm = 1;
          hasSentNoSumpTempAlarm = 0;

#ifdef debuging
          Serial.println("----------- Has not sent Sump Temp Alarm  = " + String(hasSentSumpTempAlarm));
          Serial.println("");
          delay(messageGap);
#endif
        }
      }
      else
      {
        if (hasSentNoSumpTempAlarm == 0)
        {
          core.sendMessage(ALARM_MESSAGE_ID, &errorData0);
          delay(sendMessageDelay);
          hasSentSumpTempAlarm = 0;
          hasSentNoSumpTempAlarm = 1;

#ifdef debuging
          Serial.println("Else Sump Temperture no alarm");
          Serial.println("Else Sump Temperture = " + String(SumpTemp));
          Serial.println("");
          delay(messageGap);
#endif
        }
      }
    }
  }
}