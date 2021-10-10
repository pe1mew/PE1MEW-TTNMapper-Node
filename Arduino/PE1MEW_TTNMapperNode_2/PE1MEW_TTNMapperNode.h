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

/// \file PE1MEW_TTNMApperNode.h
/// \brief TTN Mapper class to control the TTN Mapper node
/// \date 15-3-2017
/// \author Remko Welling (PE1MEW)
/// \version 1.0  Initial version

#ifndef __PE1MEW_TTNMAPPERNODE_H__
#define __PE1MEW_TTNMAPPERNODE_H__

#include "PE1MEW_TTNMapperNode_configuration.h"

#include "PE1MEW_Led.h"                 // PE1MEW_Led class
#include "PE1MEW_Button.h"              // PE1MEW_Button class
#include "TinyGPS++.h"                  // TinyGPS++ class
//#include <rn2xx3.h>                   // LoRa radio RN2xx3 class from library
#include "rn2xx3.h"                     // LoRa radio RN2xx3 class local copy
#include "RunningAverage.h"             // Running average library
#include "PE1MEW_Timer.h"

#define GPS_LED_PIN               2     ///< Pin to which the GPS LED is connected
#define STAT_LED_PIN              1     ///< Pin to which the Status LED is connected
#define ACT_LED_PIN               0     ///< Pin to which the Activity LED is connected

//#define GPS_RX_DATA_TIMEOUT_TIME  5000  ///< timeout in milliseconds for data reception of GPS (5 seconds)
//#define GPS_RX_FIX_TIMEOUT_TIME   5000  ///< timeout in mill seconds for the GPS FIX (5 seconds)
//#define TRANSMISSION_INTERVAL     10000 ///< Transmission interval in milliseconds (10 seconds) to be used with SCHEME_INTERVAL
//#define TRANSMISSION_DELAY        8000  ///< Delay between transmissions. to be used with SCHEME_REPEAT
//
//#define DEFAULT_DR                3     ///< default datarate SF9
//
//#define PAYLOAD_BUFFER_SIZE       9     ///< paylod message size

/// \brief Defines states of the TTN Mapper Node.
typedef enum {
   STATE_GPS_NODATA,    ///< No data received from GPS
   STATE_GPS_DATA,      ///< Data received from GPS but no FIX
   STATE_GPS_VALID,     ///< Data received from GPS and GPS has fix.
   STATE_RUN_TX,        ///< GPS has fix, actual position is transmitted
   STATE_RUN_PAUSE,     ///< GPS has fix but node is not moving for x minutes.
   STATE_RUN_GEOFENCE   ///< GPS has fix but coordinate is within geofence.
} eStates;

/// \brief Defines transmission scheme.
typedef enum {
   SCHEME_INTERVAL,    ///< Transmission at regular intervals
   SCHEME_REPEAT       ///< repeat transmission with fixed delay
} eSchema;

typedef struct{
  double longitude;
  double latitude;
} Coordinate_t;

/// \class PE1MEW_TTNMapperNode PE1MEW_TTNMapperNode.h <PE1MEW_TTNMapperNode.h>
/// \brief Central class that coordinates behavior of the node
class PE1MEW_TTNMapperNode
{
//variables
public:
protected:

private:

  const static int GPS_RX_DATA_TIMEOUT_TIME = 5000;  ///< timeout in milliseconds for data reception of GPS (5 seconds)
  const static int GPS_RX_FIX_TIMEOUT_TIME =  5000;  ///< timeout in mill seconds for the GPS FIX (5 seconds)
  
  const static int TRANSMISSION_INTERVAL =    10000; ///< Transmission interval in milliseconds (10 seconds) to be used with SCHEME_INTERVAL
  const static int TRANSMISSION_DELAY =       8000;  ///< Delay between transmissions. to be used with SCHEME_REPEAT
  const static int STATIC_INTERVAL_COUNT =    50;    ///< When state is static transmit after n times not transmitting send message anyway
  const static int DEFAULT_DR =               3;     ///< default datarate SF9
  const static int PAYLOAD_BUFFER_SIZE =      9;     ///< paylod message size

  /// Current state of the node
  eStates _currentState;
  
  /// Next state of the node 
  eStates _nextState;

  /// Time in milliseconds at which last data is received
  unsigned long _lastGPSDataTime;

  /// Time in milliseconds of last GPS fix.  
  unsigned long _lastGPSFixTime;          
  
  /// Time in milliseconds of last transmission.  
  unsigned long _lastTransmissionTime;    

  /// Payload buffer
  uint8_t  _txBuffer[PAYLOAD_BUFFER_SIZE];

  /// Binary representation of the latitude field of a coordinate
  uint32_t _latitudeBinary;
  
  /// Binary representation of the longitude field of a coordinate
  uint32_t _longitudeBinary;
  
  /// Altitude in meters of the node
  uint16_t _altitudeGps;
  
  /// HDOP of the GPS. HDOP represents accuracy of the coordinate
  uint8_t  _hdopGps;

  uint8_t  _moveCounter;

  bool _isMoving;

  int _staticIntervalCounter;
  
  Coordinate_t _lastCoordinate;

  eSchema _schemaType;

  /// GPS-LED (Green) for information about the GPS operation.
  PE1MEW_Led _ledGPS = PE1MEW_Led(GPS_LED_PIN);
  
  /// Status-LED (Yellow) for information about the Node operation.
  PE1MEW_Led _ledStat = PE1MEW_Led(STAT_LED_PIN);
  
  /// Activity-LED (Yellow) for information about the LoRa radio operation.
  PE1MEW_Led _ledAct = PE1MEW_Led(ACT_LED_PIN); 

  PE1MEW_Button _button = PE1MEW_Button(24);

  eStateButton _buttonState;

  /// GPS object
  TinyGPSPlus _gps = TinyGPSPlus();

  /// rn2453 object
  rn2xx3* _lora;

  RunningAverage _averageDistanceBuffer = RunningAverage(MOVING_COUNTER_MAXIMUM);

  PE1MEW_Timer timer1 = PE1MEW_Timer();
 
//functions
public:

  /// \brief Constructor
  /// \param loraObject pointer to the lora radio object.
  PE1MEW_TTNMapperNode(rn2xx3* loraObject);
  
  ~PE1MEW_TTNMapperNode();

  /// \brief Initialize LoRa radio.
  /// This function shall be called in Arduino setup() function after that
  /// both serial ports have started. 
  void initializeRadio(void);

  /// \brief Housekeeping function
  /// This function shall be called constantly from the main loop.
  /// It performs all actions in both the node and GUI.
  void process(void);
  
protected:
private:

  /// \brief Initialize TTN Mapper node.
  void initialize(void);
  
  /// \brief Indicate to process that GPS data is received
  /// This function sets the time at which the last data is received from GPS.
  void gpsDataReceived(void);
  
  /// \brief Indicate that GPS fix has detected.
  /// This function sets the time at which the last fix is detected.
  void gpsFix(void);

  /// \brief Compose application payload
  void buildPacket(void);

  /// \brief test if current position is within geofence.
  /// \return true if coordinate is within geofence, false if coordinate is outside geofence.
  bool testGeoFence(void);

  bool evaluateMoving(void);
   
}; //PE1MEW_TTNMapperNode

#endif //__PE1MEW_TTNMAPPERNODE_H__
