#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <NodeControllerCore.h>

// ---------------------------------------------------------put global variables here:---------------------------------------------------------

#define debuging

int wait = 2000; // 2 seconds
int sendMessageDelay = 100; // 1/10 second

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
#define NODE_ID 0xA2                                // Node ID in decimal is 162

#define CANOPY_TEMP_MESSAGE_ID 0xA0                  // ID in decimal is 0
#define CANOPY_TEMP_ALARM_MESSAGE_ID 0xA1            // ID in decimal is 1
bool canopyTempAlarmOnOff;
#define CANOPY_TEMP_ALARM_LOW_MESSAGE_ID 0xA2        // ID in decimal is 2
int canopyTempAlarmLow;
#define CANOPY_TEMP_ALARM_HIGH_MESSAGE_ID 0xA3       // ID in decimal is 3
int canopyTempAlarmHigh;

#define CANOPY_HUMIDITY_MESSAGE_ID 0xA4              // ID in decimal is 4
#define CANOPY_HUMIDITY_ALARM_MESSAGE_ID 0xA5        // ID in decimal is 5
bool canopyHumidityAlarmOnOff;
#define CANOPY_HUMIDITY_ALARM_LOW_MESSAGE_ID 0xA6    // ID in decimal is 6
int canopyHumidityAlarmLow;
#define CANOPY_HUMIDITY_ALARM_HIGH_MESSAGE_ID 0xA7   // ID in decimal is 7 
int canopyHumidityAlarmHigh;

#define TANK_TEMP_MESSAGE_ID 0xA8                    // ID in decimal is 8
#define TANK_TEMP_ALARM_MESSAGE_ID 0xA9              // ID in decimal is 9
bool tankTempAlarmOnOff;
#define TANK_TEMP_ALARM_LOW_MESSAGE_ID 0xAA          // ID in decimal is 10
int tankTempAlarmLow;
#define TANK_TEMP_ALARM_HIGH_MESSAGE_ID 0xAB         // ID in decimal is 11
int tankTempAlarmHigh;

#define SUMP_TEMP_MESSAGE_ID 0xC                    // ID in decimal is 12  
#define SUMP_TEMP_ALARM_MESSAGE_ID 0xAD              // ID in decimal is 13
bool sumpTempAlarmOnOff;
#define SUMP_TEMP_ALARM_LOW_MESSAGE_ID 0xAE          // ID in decimal is 14
int sumpTempAlarmLow;
#define SUMP_TEMP_ALARM_HIGH_MESSAGE_ID 0xAF         // ID in decimal is 15
int sumpTempAlarmHigh;

// ------------------------------  Variables for the temperature and humidity values to be stored in  ------------------------------

float canopyTemp = 0;
float canopyHum = 0;
float tankTemp = 0;
float sumpTemp = 0;
int tankThermometer = 0;
int sumpThermometer = 1;

// ---------------------------------------------------------put function declarations here:--------------------------------------------------------

// What to do with received messages from the CAN bus
void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data);


void SendTempHumMessage(void *parameters);

void chkTempHum();

void chkWaterTempSensors();

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

  // Initialize the OneWire communication
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize the DS18B20 temperature sensors
  WaterTempSensors.begin();

  Serial.begin(115200);

  // Initialize the SHT30-D object and pass it the I2C address
  sht30D.begin(0x44);

    xTaskCreate(
      SendTempHumMessage,   /* Task function. */
      "SendTemperatureAndHumidities", /* name of task. */
      10000,                /* Stack size of task */
      NULL,                 /* parameter of the task */
      1,                    /* priority of the task */
      NULL);                /* Task handle to keep track of created task */
                            /* pin task to core 0 */
}

//---------------------------------------------------- put your main code here, to run repeatedly:----------------------------------------------------
void loop()
{
  chkTempHum();

  chkWaterTempSensors();
}

//--------------------------------------------------------- put function definitions here: ---------------------------------------------------------

// Callback function for received messages from the CAN bus
void receive_message(uint8_t nodeID, uint16_t messageID, uint64_t data)
{
  Serial.println("Message received callback");

  // Check if the message is for this node
  if (nodeID == NODE_ID)
  {
    Serial.println("Message received to self");
    // Check the message ID for the LED control messages
    switch (messageID)
    {
      // ---------------------Demo override control messages-------------------------
    case CANOPY_TEMP_ALARM_MESSAGE_ID:
      canopyTempAlarmOnOff = data;
      Serial.println("Canopy Temperture alarm is " + String(data));
      break;

    case CANOPY_TEMP_ALARM_LOW_MESSAGE_ID:
      canopyTempAlarmLow = data;
      Serial.println("Canopy Temperture alarm low is " + String(data));
      break;

    case CANOPY_TEMP_ALARM_HIGH_MESSAGE_ID:
      canopyTempAlarmHigh = data;
      Serial.println("Canopy Temperture alarm high is " + String(data));
      break;

    case CANOPY_HUMIDITY_ALARM_MESSAGE_ID:  
      canopyHumidityAlarmOnOff = data;
      Serial.println("Canopy Humidity alarm is " + String(data));
      break;

    case CANOPY_HUMIDITY_ALARM_LOW_MESSAGE_ID:  
      canopyHumidityAlarmLow = data;
      Serial.println("Canopy Humidity alarm low is " + String(data));
      break;

    case CANOPY_HUMIDITY_ALARM_HIGH_MESSAGE_ID:
      canopyHumidityAlarmHigh = data;
      Serial.println("Canopy Humidity alarm high is " + String(data));
      break;

    case TANK_TEMP_ALARM_MESSAGE_ID:
      tankTempAlarmOnOff = data;
      Serial.println("Tank Temperture alarm is " + String(data));
      break;

    case TANK_TEMP_ALARM_LOW_MESSAGE_ID:
      tankTempAlarmLow = data;
      Serial.println("The low Temperture for the Tank alarm low is " + String(data));
      break;

    case TANK_TEMP_ALARM_HIGH_MESSAGE_ID: 
      tankTempAlarmHigh = data;
      Serial.println("The high Temperture for the Tank alarm is = " + String(data));
      break;

    case SUMP_TEMP_ALARM_MESSAGE_ID:  
      sumpTempAlarmOnOff = data;
      Serial.println("Sump Temperture alarm is " + String(data));
      break;

    case SUMP_TEMP_ALARM_LOW_MESSAGE_ID:  
      sumpTempAlarmLow = data;
      Serial.println("The low Temperture for the sump alarm = " + String(data));
      break;

    case SUMP_TEMP_ALARM_HIGH_MESSAGE_ID: 
      sumpTempAlarmHigh = data;
      Serial.println("The high Temperture for the Sump alarm = " + String(data));
      break;

    default:
      break;
    }
  }
}

//--------------------------------------

void SendTempHumMessage(void *parameters)
{
  
  while (1)
  {
    uint64_t CanopyTemp;
    uint64_t CanopyHum;
    CanopyTemp  = canopyTemp;
    CanopyHum = canopyHum;
    core.sendMessage(CANOPY_TEMP_MESSAGE_ID, &CanopyTemp); // Send the Canopy temperature
    delay(sendMessageDelay);
    core.sendMessage(CANOPY_HUMIDITY_MESSAGE_ID, &CanopyHum); // Send the Canopy humidity
    delay(sendMessageDelay);
    uint64_t TankTemp;
    uint64_t SumpTemp;
    TankTemp = tankTemp;
    SumpTemp = sumpTemp;
    core.sendMessage(TANK_TEMP_MESSAGE_ID, &TankTemp); // Send the tank temperature
    delay(sendMessageDelay);
    core.sendMessage(SUMP_TEMP_MESSAGE_ID, &SumpTemp); // Send the sump temperature
    delay(sendMessageDelay);
    }
  }


/***************************************************
  This is an example for the SHT31-D Humidity & Temp Sensor

  Designed specifically to work with the SHT31-D sensor from Adafruit
  ----> https://www.adafruit.com/products/2857

  These WaterTempSensors use I2C to communicate, 2 pins are required to
  interface
 ****************************************************/

void chkTempHum()
{
  canopyTemp = sht30D.readTemperature();
  canopyHum = sht30D.readHumidity();

#ifdef debuging
  if (!isnan(canopyTemp)) // check if 'is not a number'
  {
    Serial.print("Canopy Temp      = ");
    Serial.print(canopyTemp);
    Serial.println("°c");
    Serial.print("");
  }
  else
  {
    Serial.println("Failed to read temperature");
    Serial.print("");
  }

  if (!isnan(canopyHum)) // check if 'is not a number'
  {
    Serial.print("Canopy Humidity  = ");
    Serial.print(canopyHum);
    Serial.println("%");
    Serial.print("");
  }
  else
  {
    Serial.println("Failed to read humidity");
    Serial.print("");
  }
#endif

  delay(wait);
}

/*
//--------------------------------------------------------- Water Temperature Sensor ---------------------------------------------------------
/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com
  Based on the Dallas Temperature Library example
*********/

void chkWaterTempSensors()
{
  // Call WaterTempSensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  WaterTempSensors.requestTemperatures();

  // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  tankTemp = WaterTempSensors.getTempCByIndex(tankThermometer);

  #ifdef debuging
  Serial.print("Tank Temperature = ");
  Serial.print(tankTemp);
  Serial.println("°c");
  Serial.print("");
  #endif

  sumpTemp = WaterTempSensors.getTempCByIndex(sumpThermometer);

  #ifdef debuging
  Serial.print("Sump Temperature = ");
  Serial.print(sumpTemp);
  Serial.println("°c");
  Serial.print("");
  #endif

  delay(wait);
}