#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <NodeControllerCore.h>

// ---------------------------------------------------------put global variables here:---------------------------------------------------------

// Define the SHT30-D sensor object
Adafruit_SHT31 sht30D = Adafruit_SHT31();

// Data wire is conntec to the Arduino digital pin 4
#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature WaterTempSensors(&oneWire);

// Node controller core object
NodeControllerCore core;

bool enableHeater = false;
uint8_t loopCnt = 0;
float canopyTemp;
float canopyHum;

// ---------------------------------------------------------put function declarations here:--------------------------------------------------------

void chkTempHum();

void chkWaterTempSensors();

//------------------------------------------------------- put your setup code here, to run once:------------------------------------------------------
void setup()
{
  // Create the node controller core object
  core = NodeControllerCore();

  WaterTempSensors.begin();
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  while (!Serial)
  {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
    Serial.println("SHT30D test");
    if (!sht30D.begin(0x44))
    { // Set to 0x45 for alternate i2c addr
      Serial.println("Couldn't find SHT30D");
      while (1)
        delay(1);
    }

    Serial.print("Heater Enabled State: ");
    if (sht30D.isHeaterEnabled())
    {
      Serial.println("ENABLED");
    }
    else
    {
      Serial.println("DISABLED");
    }
  }
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

  These sensors use I2C to communicate, 2 pins are required to
  interface
 ****************************************************/
// #include <Arduino.h>

// bool enableHeater = false;
// uint8_t loopCnt = 0;

// Adafruit_SHT31 sht31 = Adafruit_SHT31();

// void setup() {
//   Serial.begin(9600);

// while (!Serial)
//   delay(10); // will pause Zero, Leonardo, etc until serial console opens

/*Serial.println("SHT31 test");
if (!sht31.begin(0x44))
{ // Set to 0x45 for alternate i2c addr
  Serial.println("Couldn't find SHT31");
  while (1)
    delay(1);
}

Serial.print("Heater Enabled State: ");
if (sht31.isHeaterEnabled())
  Serial.println("ENABLED");
else
  Serial.println("DISABLED");
}
*/
// void loop()
//{
//   float t = sht31.readTemperature();
//  float h = sht31.readHumidity();

void chkTempHum()
{
  canopyTemp = sht30D.readTemperature();
  canopyHum = sht30D.readHumidity();
  if (!isnan(canopyTemp))
  { // check if 'is not a number'
    Serial.print("Temp *C = ");
    Serial.print(canopyTemp);
    Serial.print("\t\t");
  }
  else
  {
    Serial.println("Failed to read temperature");
  }

  if (!isnan(canopyHum))
  { // check if 'is not a number'
    Serial.print("Hum. % = ");
    Serial.println(canopyHum);
  }
  else
  {
    Serial.println("Failed to read humidity");
  }

  delay(1000);

  // Toggle heater enabled state every 30 seconds
  // An ~3.0 degC temperature increase can be noted when heater is enabled
  if (loopCnt >= 30)
  {
    enableHeater = !enableHeater;
    sht30D.heater(enableHeater);
    Serial.print("Heater Enabled State: ");
    if (sht30D.isHeaterEnabled())
      Serial.println("ENABLED");
    else
      Serial.println("DISABLED");

    loopCnt = 0;
  }
  loopCnt++;
}

//--------------------------------------------------------- Water Temperature Sensor ---------------------------------------------------------
/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com
  Based on the Dallas Temperature Library example
*********/

// #include <OneWire.h>
// #include <DallasTemperature.h>

// Data wire is conntec to the Arduino digital pin 4
// #define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices
// OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor
// DallasTemperature sensors(&oneWire);

// void setup(void)
//{
//  Start serial communication for debugging purposes
//  Serial.begin(9600);
// Start up the library
//  sensors.begin();
//}

void chkWaterTempSensors()
{
  // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  WaterTempSensors.requestTemperatures();

  Serial.print("Celsius temperature: ");
  // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  Serial.print(WaterTempSensors.getTempCByIndex(0));
  Serial.print(" - Fahrenheit temperature: ");
  Serial.println(WaterTempSensors.getTempFByIndex(0));
  delay(1000);
}