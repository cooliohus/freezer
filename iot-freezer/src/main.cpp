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

// edit the config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients - eric.
//
#include "config.h"   // configuration stuff
#include <OneWire.h>
#include <DallasTemperature.h>

// defines for setting the counter for periodic interrupts
unsigned long maxDelay = 26843542;
#define oneTick 3.2                    // 3.2 uS per tick - 80MHz clock  DIV_256
#define oneSecond 1000000 / oneTick    // one second (1000000 uS / tick)
#define IODELAY 15                     // generate interrupt every 15 seconds

boolean state = false;
#define high_on 32.0
#define low_off 25.0

unsigned long mydelay;

// these variables are used by the interrupt handler.  Good practices
// to declare them volatile so the compiler doesn't optimize them away
volatile boolean gotInt = false;
volatile int intCnt = 0;
volatile bool gotReply = true;

// set up the adafruitio feeds.  Note that the object iox was created in config.h
AdafruitIO_Feed *feedambient = iox.feed("ambienttemp");
AdafruitIO_Feed *feedfreezer = iox.feed("freezertemp");

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 9
#define ON_LED 16

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress devambient = { 0x28, 0xFF, 0x1D, 0xCE, 0x39, 0x20, 0x01, 0x3B };
DeviceAddress devfreezer = { 0x28, 0xD8, 0x19, 0xCD, 0x39, 0x20, 0x01, 0x9F };

//DeviceAddress devfreezer = { 0x28, 0xC6, 0xB0, 0xD5, 0x39, 0x20, 0x01, 0x33 };


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
// sent to ada io, it was attached to the 'digital' feed in
// the setup() function below.

void handleMessage(AdafruitIO_Data *data) {

  gotReply = true;

  if (strcmp((char *)data->feedName(),"freezertemp")== 0) {
    Serial.print("ack from freezertemp: ");
    Serial.println(data->toChar());
  }
  else if (strcmp((char *)data->feedName(),"ambienttemp")== 0) {
    Serial.print("ack from ambienttemp: ");
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
  while(! Serial);
    delay(100);

  delay(1000);  // wait for serial port to fully initialize
  
  // connect to io.adafruit.comhttp://coolioh.com/
  Serial.print("Connecting to Adafruit IO");
  iox.connect();   // connect and sign in to adaio
  
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
  feedfreezer->onMessage(handleMessage);
  feedambient->onMessage(handleMessage);
  
  Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the sensor library
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

    // show the addresses we found on the bus
  DeviceAddress inx0, inx1;
  if (!sensors.getAddress(inx0, 0)) Serial.println("Unable to find address for Device inx0");
  if (!sensors.getAddress(inx1, 1)) Serial.println("Unable to find address for Device inx1");

  Serial.print("inx0 0 Address: ");
  printAddress(inx0);
  Serial.println();
  
  Serial.print("inx1 0 Address: ");
  printAddress(inx1);
  Serial.println();

  Serial.print("Ambient sensor address: ");
  printAddress(devambient);
  Serial.println();

  Serial.print("Freezer sensor address: ");
  printAddress(devfreezer);
  Serial.println();http://coolioh.com/

  // set the resolution to 9 bit per device
  
  sensors.setResolution(devambient, TEMPERATURE_PRECISION);
  sensors.setResolution(devfreezer, TEMPERATURE_PRECISION);

  Serial.print("Device ambient Resolution: ");
  Serial.print(sensors.getResolution(devambient), DEC);
  Serial.println();

  Serial.print("Device freezer Resolution: ");
  Serial.print(sensors.getResolution(devfreezer), DEC);
  Serial.println();

// builtin LED
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH); // turn LED off

  pinMode(ON_LED,OUTPUT);
  digitalWrite(ON_LED,LOW);
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
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
    
    gotInt = false;
    intCnt = 0;

    // call sensors.requestTemperatures() to issue a global temperature
    // request to all devices on the bus
    Serial.print("Requesting temperatures...");
    sensors.requestTemperatures();
    Serial.println("DONE");

    // print the device information
    printData(devambient);
    printData(devfreezer);

    //printData(outsideThermometer);

    float tempC = sensors.getTempC(devfreezer);
    
    if (state and (DallasTemperature::toFahrenheit(tempC) < low_off)) {
      digitalWrite(ON_LED, LOW);
      state = false;
      }
    else if (!state and (DallasTemperature::toFahrenheit(tempC) > high_on)) {
      digitalWrite(ON_LED, HIGH);
      state = true;
    }
  
    feedfreezer->save(DallasTemperature::toFahrenheit(tempC));

    tempC = sensors.getTempC(devambient);
    feedambient->save(DallasTemperature::toFahrenheit(tempC));

    
    digitalWrite(LED_BUILTIN,HIGH);  // turn LED off
   
  } // end of process timer interupt

} // end of loop
