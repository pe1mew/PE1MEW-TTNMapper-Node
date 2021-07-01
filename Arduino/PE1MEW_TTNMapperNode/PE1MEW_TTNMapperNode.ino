 /*--------------------------------------------------------------------
  This file is part of the PE1MEW TTNMapper node.
  
  The PE1MEW TTNMapper node is free software: 
  you can redistribute it and/or modify it under the terms of a Creative 
  Commons Attribution-NonCommercial 4.0 International License 
  (http://creativecommons.org/licenses/by-nc/4.0/) by 
  PE1MEW (http://pe1mew.nl) E-mail: pe1mew@pe1mew.nl

  The PE1MEW TTNMapper node is distributed in the hope that 
  it will be useful, but WITHOUT ANY WARRANTY; without even the 
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
  PURPOSE.
  --------------------------------------------------------------------*/

/// \file PE1MEW_TTNMapperNode.ino
/// \brief PE1MEW TTN Mapper node
/// \date 15-3-2017
/// \author Remko Welling (PE1MEW)
/// \version 1.3  12- 1-2019 Added transmission during static operation inhibit
/// \version 1.2  18-09-2018 Additions for static behaviour
/// \version 1.1  15-09-2018 Modification SF to SF9,
///                          Correction of statemachine in PE1MEW_TTNMapperNode
/// \version 1.0  15-03-2018 Initial version



/*!
 * 
 \mainpage TTN Mapper node with GUI by PE1MEW

 This node is designed for usage on The Things Network (TTN) (See: https://thethingsnetwork.org) and with
 TTN Mapper (http://ttnmapper.org/).

 The purpose of the node is to send position information at regular intervals so TTN Mapper can 
 read the fieldstrenghth measured by the receiving gateway(s) at which the packets are received by the TTN gateways. 
 This information is than plotted on a map to display the coverage of the TTN network.

 The node is equipped with a GUI to indicate status and activities to the operator. Also the node can be reset
 and a transmission can be initiated. 

 \par Credits

 This program is based on the original program by JP Meijers and contains portions of the original code.
 His code can be found at: https://github.com/jpmeijers/RN2483-Arduino-Library 
 The original code was published under the Apache License, Version 2.0 (the "License").
 
 \par Overview

 This program is for any Arduino compatible board or microprocessor that has 2 serial ports in combination 
 with a GPS that generates NMEA compatible data. Where the original program was using softserial, this program is using 
 hardware serial. 

 Coordinates from the GPS is packed into a payload using binary encoding, and sent using the RN2xx3 module.
 Transmissions are done at a regular interval while still keeping to the 1% duty cycle rules enforced by the 
 RN2483's built in LoRaWAN stack. Even though this is allowed by the radio regulations of the 868MHz band, 
 the fair use policy of TTN may prohibit this.

 \par Features

 - GUI to indicate operations to operator
 - Use universal GPS receiver
 - Detect GPS data reception
 - Detect GPS Fix
 - Send position to TTN Mapper over the TTN Network
 - Inhibits sending position when within geofence
 - Geofence inhibit can be manually overruled

 \par Use the program and contribute to TTN Mapper.

 To contribute to TTN Mapper follow these steps: 
 1. Register a new Application and/or new device at TTN. The dashboard is at https://console.thethingsnetwork.org .
    If required register yourself as a new user.
    Make sure that your device is using ABP and that "relax frame count" is enabled.

 2. Copy all inormation from the console at TTN. The url is formatted as: 
    \code 
    https://console.thethingsnetwork.org/applications/<applicationName>/devices/<unitName> 
    \endcode
     Retrieve the following information:
  - Device address
  - Network Session key
  - Application Session key

 3. Copy the information of your device in to the init function in file PE1MEW_TTNMapperNode.cpp:
 \code
   join_result = _lora->initABP("xxxxxxxx", "yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy", "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz");
 \endcode
 4. Make sure packets are arriving on the TTN console when your node is powered and in reach of a gateway.
 5. Share your Application ID, Access Key and Device ID with contribute@ttnmapper.org so that your measurements 
    can be automatically imported into TTN Mapper.

 Optional:
 1. Disable waitloop: Remove the waitloop at initialisation of serial port to speed up after powerering the node:
 Comment out or remove the line in PE1MEW_TTNMapperNode.ino to disable waitloop: 
 \code 
   while ((!Serial) && (millis() < 10000));
 \endcode
 2. Activate geofencing:
 Add centre coordinate of geofence to the code in PE1MEW_TTNMapperNode.h:
 \code
 // \brief Defines geofence coordinates
 // These coordinates are the center of a circle 
 // The number of coordinates shall be the same as the COORDINATES_COUNT define!
 const PROGMEM double coordinates[3][2] = { 
   {yy.yyyyy, x.xxxxx},  //< Coordinate 1
   {yy.yyyyy, x.xxxxx},  //< Coordinate 2
   {yy.yyyyy, x.xxxxx}   //< Coordinate 3
 };
 \endcode
 3. Decode binary payload in TTN: 
 To decode the binary payload in the TTN console, you can use the following javascript decoder function.
 This function will also split the fields in to topics in MQTT.
 \code
 function Decoder(bytes, port) {
   // Decode an uplink message from a buffer
   // (array) of bytes to an object of fields.
   var decoded = {};
 
   // if (port === 1) decoded.led = bytes[0];
   decoded.lat = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
   decoded.lat = (decoded.lat / 16777215.0 * 180) - 90;
 
   decoded.lon = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
   decoded.lon = (decoded.lon / 16777215.0 * 360) - 180;
 
   var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
   var sign = bytes[6] & (1 << 7);
   if(sign){
     decoded.alt = 0xFFFF0000 | altValue;
   }else{
     decoded.alt = altValue;
   }
 
   decoded.hdop = bytes[8] / 10.0;
 
   return decoded;
 }
  
 \endcode

 \par User interface.
 
 The PE1MEW TTN Mapper node has 4 LED and 2 switches that can fysically be arranged in any way you want. 
 The following picture shows a example of how teh user interface could look like:
 \image html PE1MEW_TTNMapperNodeGUIFront.png "PE1MEW TTN Mapper node user interface" width=600px

 \par Inputs:

 - Activity button: The following function(s) have been implemented:
  - Press button > 4 seconds: If geofence inhibit is active a postion transmission is intiated. Else there is no function.
  - When transmission is inhibited due to static operation, presing ACT will initiate a transmission 
 - Reset button: This button is hardwired to the Arduino en will reset the microprocessor.
 
 \par Ouputs:

 In general the led colors have a specific meaning:
 - Green: Information; operation OK
 - Yellow: Information; system is operational information and errors.
 - Red (Not used): Failure.
 
 The following leds have been implemented:
 - Power LED: This led is not controlled by software. It is connected directly to the 3,3 V power of the Arduino node.
 - GPS LED: This led has 3 states:
  - OFF: No data is received from the GPS.
  - Blinking: Data is received from GPS but no FIX.
  - ON: Data is received and GPS has fix.
 - Status LED: This led has 3 states:
  - OFF: No Information availabale.
  - ON: Last location is within range of the geofence, position transmissions are inhibited.
  - Blinking fast: Initialisation of LoRa radio has failed. 
 - Activity LED: This led has 2 states:
  - OFF: No information availabale.
  - Blinking fast: transmission is inhibited due to static operation
  - ON: Position is transmitted.

 \par Connectors:
 
 - GPS antenna: 
 - LoRa antenna:

 */

// Includes

#include "PE1MEW_TTNMapperNode.h"
//#include <rn2xx3.h>                 // LoRa radio RN2xx3 class from libraries
#include "rn2xx3.h"                 // LoRa radio RN2xx3 class local copy

// Create objects

/// lora object linked to Serial1.
rn2xx3 myLora = rn2xx3(Serial1);    // LoRa Radio connected to serial 1

PE1MEW_TTNMapperNode ttnMapperNode = PE1MEW_TTNMapperNode(&myLora);

/// \brief setup for Arduino
/// This function configures all parameters and functions that cannot be configured else where.
/// Configured are:
/// - Both serial ports Serial and Serial1
/// - LoRa Radio after that the serial ports are initialized.
void setup() {
  Serial.begin(4800);   //serial to computer (TX) and GPS (RX)
  Serial1.begin(57600); //serial to RN2xx3

  // make sure usb serial connection is available,
  // or after 10s go on anyway for 'headless' use of the
  // node.
  while ((!Serial) && (millis() < 10000));

  ttnMapperNode.initializeRadio();  // initialize LoRa radio in PE1MEW_TTNMapperNode object.
}

/// \brief loop for Arduino
/// This function is the main loop for Arduino.
/// Only the process function is called from the PE1MEW_TTNMapperNode object for house keeping.
void loop() {
  ttnMapperNode.process();
}
