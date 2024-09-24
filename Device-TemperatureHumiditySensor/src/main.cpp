#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <NodeControllerCore.h>

// ---------------------------------------------------------put global variables here:---------------------------------------------------------

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

// arrays to hold device addresses
DeviceAddress tankThermometer, sumpThermometer;

// Node controller core object
//NodeControllerCore core;

bool enableHeater = false;
uint8_t loopCnt = 0;
float canopyTemp = 0;
float canopyHum = 0;
float tankTemp = 0;
float sumpTemp = 0;

// ---------------------------------------------------------put function declarations here:--------------------------------------------------------

void chkTempHum();

void chkWaterTempSensors();

void printAddress(DeviceAddress deviceAddress);

void printData(DeviceAddress deviceAddress);

void printTemperature(DeviceAddress deviceAddress);

void printResolution(DeviceAddress deviceAddress);



//------------------------------------------------------- put your setup code here, to run once:------------------------------------------------------
void setup()
{
  // Create the node controller core object
  //core = NodeControllerCore();

  
  Wire.begin(I2C_SDA, I2C_SCL);
  WaterTempSensors.begin();
  Serial.begin(115200);

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

 // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(WaterTempSensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (WaterTempSensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // Assign address manually. The addresses below will beed to be changed
  // to valid device addresses on your bus. Device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // WaterTempSensors.getAddress(deviceAddress, index)
  //tankThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };
  //sumpThermometer   = { 0x28, 0x3F, 0x1C, 0x31, 0x2, 0x0, 0x0, 0x2 };

  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then 
  // use those addresses and manually assign them (see above) once you know 
  // the devices on your bus (and assuming they don't change).
  // 
  // method 1: by index
  if (!WaterTempSensors.getAddress(tankThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  if (!WaterTempSensors.getAddress(sumpThermometer, 1)) Serial.println("Unable to find address for Device 1"); 

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(tankThermometer);
  Serial.println();

  Serial.print("Device 1 Address: ");
  printAddress(sumpThermometer);
  Serial.println();

  // set the resolution to 9 bit per device
  WaterTempSensors.setResolution(tankThermometer, TEMPERATURE_PRECISION);
  WaterTempSensors.setResolution(sumpThermometer, TEMPERATURE_PRECISION);

  Serial.print("Device 0 Resolution: ");
  Serial.print(WaterTempSensors.getResolution(tankThermometer), DEC); 
  Serial.println();

  Serial.print("Device 1 Resolution: ");
  Serial.print(WaterTempSensors.getResolution(sumpThermometer), DEC); 
  Serial.println();
  
}

//---------------------------------------------------- put your main code here, to run repeatedly:----------------------------------------------------
void loop()
{
  chkTempHum();
  chkWaterTempSensors();
    // call WaterTempSensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  WaterTempSensors.requestTemperatures();
  Serial.println("DONE");

  // print the device information
  //printData(tankThermometer);
  //printData(sumpThermometer);
}


//--------------------------------------------------------- put function definitions here: ---------------------------------------------------------

/***************************************************
  This is an example for the SHT31-D Humidity & Temp Sensor

  Designed specifically to work with the SHT31-D sensor from Adafruit
  ----> https://www.adafruit.com/products/2857

  These WaterTempSensors use I2C to communicate, 2 pins are required to
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
  if (! isnan(canopyTemp))  // check if 'is not a number'
  { 
    Serial.print("Temp *C = ");
    Serial.print(canopyTemp);
    Serial.print("\t\t");
  }
  else
  {
    Serial.println("Failed to read temperature");
  }

  if (!isnan(canopyHum))  // check if 'is not a number'
  {
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
// DallasTemperature WaterTempSensors(&oneWire);

// void setup(void)
//{
//  Start serial communication for debugging purposes
//  Serial.begin(9600);
// Start up the library
//  WaterTempSensors.begin();
//}

 
void chkWaterTempSensors()
{
  // Call WaterTempSensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  WaterTempSensors.requestTemperatures();

  Serial.print("Celsius temperature: ");
  // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  Serial.print(WaterTempSensors.getTempCByIndex(0));
  Serial.print(" - Fahrenheit temperature: ");
  Serial.println(WaterTempSensors.getTempFByIndex(0));
  delay(1000);
}


// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = WaterTempSensors.getTempC(deviceAddress);
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
}

// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress)
{
  Serial.print("Resolution: ");
  Serial.print(WaterTempSensors.getResolution(deviceAddress));
  Serial.println();    
}

// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  printTemperature(deviceAddress);
  Serial.println();
}


