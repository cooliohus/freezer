#include <Arduino.h>

 // Adafruit IO Digital Input Example
// Tutorial Link: https://learn.adafruit.com/adafruit-io-basics-digital-input
//
// Modified by Andy Cooper
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Written by Todd Treece for Adafruit Industries
// Copyright (c) 2016 Adafruit Industries
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

/************************** Configuration ***********************************/

/*
 * Library versions known to work
 * ----------------------------- 
 * 
 * Adafruit ESP8266 v1.0.0
 * AdafruitIO v2.7.2
 * Adafruit MQTT v0.20.1
 *
*/
// edit the config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.
//
#include "config.h"   // configuration stuff

#include <OneWire.h>
#include <DallasTemperature.h>

//#include <Wire.h>
//#include <SPI.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BME280.h>
//#include <DHT.h>

// defines for temp / humidity sensor
//#define DHTPIN 12
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

// defines for setting the counter for periodic interrupts
unsigned long maxDelay = 26843542;
#define oneTick 3.2                    // 3.2 uS per tick - 80MHz clock  DIV_256
#define oneSecond 1000000 / oneTick    // one second (1000000 uS / tick)
#define IODELAY 15                     // generate interrupt every 15 seconds

unsigned long counter = 0;
unsigned long ftempcnt = 0;
unsigned long fhumcnt = 0;
int vin;
float vout;


boolean state = false;
#define high_on 0.0
#define low_off -5.0

unsigned long mydelay;

// these variables are used by the interrupt handler.  Good practices
// to declare them volatile so the compiler doesn't optimize them away
volatile boolean gotInt = false;
volatile int intCnt = 0;
volatile bool gotReply = true;

// set up the adafruitio feeds.  Note that the object iox was created in config.h
AdafruitIO_Feed *ffreezer = iox.feed("freezertemp");
// AdafruitIO_Feed *fouttemp = iox.feed("outlettemp");

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 9

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress tfreezer;

// Assign address manually. The addresses below will need to be changed
// to valid device addresses on your bus. Device address can be retrieved
// by using either oneWire.search(deviceAddress) or individually via
// sensors.getAddress(deviceAddress, index)
// DeviceAddress insideThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };
// DeviceAddress outsideThermometer   = { 0x28, 0x3F, 0x1C, 0x31, 0x2, 0x0, 0x0, 0x2 };

void printTemperature(DeviceAddress deviceAddress);
void printAddress(DeviceAddress deviceAddress);



// This function is called each time the interval timer times out
//
void IRAM_ATTR myIsrTimer() {
  //portENTER_CRITICAL_ISR(&timerMux);
  gotInt = true;   // tell outside world a timer interrupt occurs
  gotReply = true; // dead man switch for adafruit io
  //digitalWrite(myLED, !(digitalRead(myLED)));   // for debugging
  //portEXIT_CRITICAL_ISR(&timerMux);
  intCnt++;
}

// this function is called whenever a 'digital' feed message
// is received from Adafruit IO acknowledging receipt of messages
// send to ada io, it was attached to the 'digital' feed in
// the setup() function below.  ... and this function has grown,
// needs optimization :-(

void handleMessage(AdafruitIO_Data *data) {

  gotReply = true;

  if (strcmp((char *)data->feedName(),"freezertemp")== 0) {
    ftempcnt += 1;
    Serial.print("ack from freezertemp: ");
    Serial.println(data->toChar());
  }
  else {
    Serial.print("unknown message from: ");
    Serial.print(data->feedName()); Serial.print("  ");
    Serial.println(data->toChar());
  }
}



void setup() {

  // start the serial connection
  Serial.begin(115200);
  delay(500);
  
  // wait for serial monitor to open
  while(! Serial);
    delay(100);

  // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");

  iox.connect();   // connect and sign in to adaio

  // wait for a connection
  while(iox.status() < AIO_CONNECTED) {
    delay(500);
    Serial.print("."); 
  }

  // we are connected
  Serial.println();
  Serial.println(iox.statusText());

  // intitialize the timer to interrupt every mydelay periods
  // see IODELAY define

  mydelay = oneSecond * IODELAY;  // period between samples
  timer1_isr_init();
  timer1_disable();
  timer1_attachInterrupt(myIsrTimer);
  timer1_enable(TIM_DIV256, TIM_EDGE, /*TIM_SINGLE*/TIM_LOOP);
  timer1_write(mydelay);
  
  // setup message handlers - all messages received from ada io are directed to the handleMessage function
  // except handle lights because we want to do something - it is easier to process in a separate function
  ffreezer->onMessage(handleMessage);
  //fouttemp->onMessage(handleMessage);
  
 Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin();

  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then
  // use those addresses and manually assign them (see above) once you know
  // the devices on your bus (and assuming they don't change).
  //
  // method 1: by index
  if (!sensors.getAddress(tfreezer, 1)) Serial.println("Unable to find address for Device 0");
  //if (!sensors.getAddress(outsideThermometer, 0)) Serial.println("Unable to find address for Device 1");

  // method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices,
  // or you have already retrieved all of them. It might be a good idea to
  // check the CRC to make sure you didn't get garbage. The order is
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to insideThermometer
  //if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");
  // assigns the seconds address found to outsideThermometer
  //if (!oneWire.search(outsideThermometer)) Serial.println("Unable to find address for outsideThermometer");

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(tfreezer);
  Serial.println();

  //Serial.print("Device 1 Address: ");
  //printAddress(outsideThermometer);
  //Serial.println();

  // set the resolution to 9 bit per device
  
  sensors.setResolution(tfreezer, TEMPERATURE_PRECISION);
  //sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);

  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(tfreezer), DEC);
  Serial.println();

  //Serial.print("Device 1 Resolution: ");
  //Serial.print(sensors.getResolution(outsideThermometer), DEC);
  //Serial.println();

// builtin LED
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH); // turn LED off

  pinMode(16,OUTPUT);
  digitalWrite(16,LOW);



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
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
}

// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress)
{
  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress));
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

void loop() {

  iox.run();

  if (intCnt > 0) {  // Period is governed by timer interrupt handler.  For times > max delay
                     // set comparison to an integer multiple of mydelay.  E.G. if mydelay is
                     // 15 seconds the intCnt > 19 is 5 minutes
    
    digitalWrite(LED_BUILTIN,LOW); // turn LED on
    // digitalWrite(16,HIGH);  //turn on switch
    gotInt = false;
    intCnt = 0;

    // call sensors.requestTemperatures() to issue a global temperature
    // request to all devices on the bus
    Serial.print("Requesting temperatures...");
    sensors.requestTemperatures();
    Serial.println("DONE");

    // print the device information
    printData(tfreezer);
    //printData(outsideThermometer);

    float tempC = sensors.getTempC(tfreezer);
    
    if (state and (DallasTemperature::toFahrenheit(tempC) < low_off)) {
      digitalWrite(16, LOW);
      state = false;
      }
    else if (!state and (DallasTemperature::toFahrenheit(tempC) > high_on)) {
      digitalWrite(16, HIGH);
      state = true;
    }
  
    ffreezer->save(DallasTemperature::toFahrenheit(tempC));

    digitalWrite(LED_BUILTIN,HIGH);  // turn LED off
    //digitalWrite(16,LOW);  // turn switch off
   
  } // end of process timer interupt

} // end of loop