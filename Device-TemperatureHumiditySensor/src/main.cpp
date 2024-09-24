#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <NodeControllerCore.h>

// ---------------------------------------------------------put global variables here:---------------------------------------------------------

int wait = 5000; // 5 seconds

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

bool enableHeater = false;
uint8_t loopCnt = 0;
float canopyTemp = 0;
float canopyHum = 0;
float tankTemp = 0;
float sumpTemp = 0;
int tankThermometer = 0;
int sumpThermometer = 1;

// ---------------------------------------------------------put function declarations here:--------------------------------------------------------

void chkTempHum();

void chkWaterTempSensors();

//------------------------------------------------------- put your setup code here, to run once:------------------------------------------------------
void setup()
{
  // Create the node controller core object
  core = NodeControllerCore();
  // Initialize the OneWire communication
  Wire.begin(I2C_SDA, I2C_SCL);
  // Initialize the Ds18 sensor
  WaterTempSensors.begin();
  Serial.begin(115200);

  sht30D.begin(0x44);
}

//---------------------------------------------------- put your main code here, to run repeatedly:----------------------------------------------------
void loop()
{
  chkTempHum();

  chkWaterTempSensors();
}

//--------------------------------------------------------- put function definitions here: ---------------------------------------------------------

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
  if (!isnan(canopyTemp)) // check if 'is not a number'
  {
    Serial.print("Canopy Temp = ");
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
    Serial.print("Canopy Humidity = ");
    Serial.print(canopyHum);
    Serial.println("%");
  Serial.print("");
  }
  else
  {
    Serial.println("Failed to read humidity");
  Serial.print("");
  }
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
  Serial.print("Tank Temperature = ");
  Serial.print(tankTemp);
  Serial.println("°c");
  Serial.print("");
  sumpTemp = WaterTempSensors.getTempCByIndex(sumpThermometer);
  Serial.print("Sump Temperature = ");
  Serial.print(sumpTemp);
  Serial.println("°c");
  Serial.print("");
  delay(wait);
}