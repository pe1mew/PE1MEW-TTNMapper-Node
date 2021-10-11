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

/// \file PE1MEW_TTNMApperNode.cpp
/// \brief TTN Mapper class to control the TTN Mapper node
/// \date 1-7-2021
/// \author Remko Welling (PE1MEW)
/// \version See revision table
///
/// ## revision table
/// Version | Comment
/// --------|-----------------------------------
/// 1.0     | Initial version
/// 1.1     | Added functionality to stop transmitting when node is not moving. Removed static ABP parameters and moved to file PE1MEW_TTNMapper_configuration.h
/// 1.2     | Added functionality to periodically transmit when node is not moving
/// 1.3     | Added define for mode selection ABP or OTAA, changed code-style.

#include "PE1MEW_TTNMapperNode.h"

#include <arduino.h> 

#include "TinyGPS++.h"  // TinyGPS++ class
//#include <rn2xx3.h>   // LoRa radio RN2xx3 class from library
#include "rn2xx3.h"     // LoRa radio RN2xx3 class local copy


PE1MEW_TTNMapperNode::PE1MEW_TTNMapperNode(rn2xx3* loraObject):
  _currentState(STATE_GPS_DATA),
  _nextState(STATE_GPS_NODATA),
  _lastGPSDataTime(0),
  _lastGPSFixTime(0),
  _lastTransmissionTime(0),
  _latitudeBinary(0),
  _longitudeBinary(0),
  _altitudeGps(0),
  _hdopGps(0),
  _moveCounter(0),
  _isMoving(true),
  _dataRate(0),
  _staticIntervalCounter(STATIC_INTERVAL_COUNT),
  _schemaType(SCHEME_INTERVAL),
  _buttonState(STATE_NOT_PRESSED),
  _lora(loraObject)  
{
  initialize();
} //PE1MEW_TTNMapperNode

// default destructor
PE1MEW_TTNMapperNode::~PE1MEW_TTNMapperNode(){
} //~PE1MEW_TTNMapperNode

void PE1MEW_TTNMapperNode::initialize(void){
  // set time for timeout timers
  unsigned long time = millis();
  _lastGPSDataTime = time;
  _lastGPSFixTime = time;
  _lastTransmissionTime = time;

  // blink leds to indicate working
  _ledGPS.startBlink(1); // 2 blinks at period time of 500 ms
  _ledStat.startBlink(2);
  _ledAct.startBlink(3);

  // explicitly start with clean average buffer
  _averageDistanceBuffer.clear();
}

void PE1MEW_TTNMapperNode::process(void){
   // process all I/O in the GUI
  _ledGPS.process();
  _ledStat.process();
  _ledAct.process();
  _buttonState = _button.process();
  
  // Test for GPS FIX timeout.
  if (_gps.location.age() < GPS_RX_FIX_TIMEOUT_TIME){
    gpsFix(); // set time at which FIX was detected.
  }

  // Test for GPS data available.
  // When available send character to GPS object and set time.
  while (Serial.available()){
    char c = Serial.read();
    _gps.encode(c);
    gpsDataReceived();
  }

  // get current time for usage in this function.
  // this prevents excessive use of the millis() function.
  unsigned long currentTime = millis();

  // pre-process next state.
  // Make all required changes to the GUI.
  if (_currentState != _nextState){
    // Process new state (current)
    switch (_nextState){
      
      case STATE_GPS_NODATA:
        _ledGPS.setOff();
      break;
        
      case STATE_GPS_DATA:
        _ledGPS.startBlink(true, 1, 500);
      break;
        
      case STATE_GPS_VALID:
        _ledGPS.setOn();
        _ledStat.setOff();
      break;
  
      case STATE_RUN_TX:
        _ledAct.setOn();
      break;

      case STATE_RUN_PAUSE:
        _ledAct.startBlink(true, 1, 100);
      break;
      
      case STATE_RUN_GEOFENCE:
        _ledStat.setOn();
      break;
      
      default:

      break;
    } // end switch
  } // end if
  
  // switch current state to next state
  _currentState = _nextState;
    
  // Process current state activities.
  // This switch case is processing timeout timers
  switch (_currentState){
    
      case STATE_GPS_NODATA:
        if (currentTime - _lastGPSDataTime < GPS_RX_DATA_TIMEOUT_TIME){
          _nextState = STATE_GPS_DATA;
        }
      break;

      case STATE_GPS_DATA:
        if (currentTime - _lastGPSDataTime > GPS_RX_DATA_TIMEOUT_TIME){
          _nextState = STATE_GPS_NODATA;
        }else  if (currentTime - _lastGPSFixTime < GPS_RX_FIX_TIMEOUT_TIME){
          _nextState = STATE_GPS_VALID;
        }
      break;
        
      case STATE_GPS_VALID:
        if (currentTime - _lastGPSFixTime > GPS_RX_FIX_TIMEOUT_TIME){
          _nextState = STATE_GPS_DATA;
          
        }else if (testGeoFence()){
          // Node is located within geofence.
          _nextState = STATE_RUN_GEOFENCE;
          
        }else if ((currentTime - _lastTransmissionTime > TRANSMISSION_INTERVAL) && (!_isMoving) && (_schemaType == SCHEME_DISTANCE)){
          _nextState = STATE_RUN_PAUSE;
          _lastTransmissionTime = currentTime;
          
        }else if ((currentTime - _lastTransmissionTime > TRANSMISSION_INTERVAL) && (_isMoving) && (_schemaType == SCHEME_DISTANCE)){
          _nextState = STATE_RUN_TX;
          _lastTransmissionTime = currentTime;
          
        }else if ((currentTime - _lastTransmissionTime > TRANSMISSION_DELAY) && (!_isMoving) && (_schemaType == SCHEME_INTERVAL)){
          // Node is not moving: During static operation allow periodic transmissions
          if(_staticIntervalCounter > 0){
            _nextState = STATE_RUN_PAUSE;
            _staticIntervalCounter--;
          }else{
            _nextState = STATE_RUN_TX;
            _staticIntervalCounter = STATIC_INTERVAL_COUNT;  // reset interval counter
          }
          _lastTransmissionTime = currentTime;
          
        }else if ((currentTime - _lastTransmissionTime > TRANSMISSION_DELAY) && (_isMoving) && (_schemaType == SCHEME_INTERVAL)){
          _nextState = STATE_RUN_TX;
          _lastTransmissionTime = currentTime;
        }
      break;
  
      case STATE_RUN_TX:
        _ledStat.setOff();
        // send message
        buildPacket();  // prepare payload

        /*
         * This portion of code is to cyclic set a data rate starting at DR 0 (SF12)
         * to DR 5 (SF7).
         * - First test is actual datarate is a value from 0 to 5.
         * - If data rate is > 5 reset to 0.
         * - Send message and increment data rate for next transmission.
         */
        if(_dataRate > DATA_RATE_MAXIUM){
          _dataRate = DATA_RATE_MINIMUM;
        }
        _lora->setDR(_dataRate++);

        
        _lora->txBytes(_txBuffer, sizeof(_txBuffer)); // transmit payload
        _ledAct.setOff();
        evaluateMoving();
                
        if ( _schemaType == SCHEME_INTERVAL){
          _lastTransmissionTime = millis();
        }

        _nextState = STATE_GPS_VALID; // return to idle state

      break;

      case STATE_RUN_PAUSE:
        // \todo add system pause here

        // Try to set timer with 2000 ms
        // if true, set is success; continue
        // if false; timer was set before continue
        if(!timer1.set(2000)){
          // when timer expired do work:
          if(timer1.getExpired()){
            _ledAct.setOff();
            // housekeeping for detection of static behaviour
            evaluateMoving();
                    
            if ( _schemaType == SCHEME_INTERVAL){
              _lastTransmissionTime = millis();
            }
            _nextState = STATE_GPS_VALID; // return to idle state
            
          }else if (_buttonState == STATE_PRESSED){ // test for button press to override pause state and send packet
            _nextState = STATE_RUN_TX;
            _button.reset(); // reset button evaluation
          }
        }
        
      break;
      
      case STATE_RUN_GEOFENCE:
        if (currentTime - _lastGPSFixTime > GPS_RX_FIX_TIMEOUT_TIME){
          _nextState = STATE_GPS_DATA;
        }else if (!testGeoFence()){
          _nextState = STATE_GPS_VALID;
        }else if (_buttonState == STATE_PRESSED_T1){
          _nextState = STATE_RUN_TX;
          _button.reset(); // reset button evaluation
        }
      break;
      
      default:

      break;
  } // end switch
  
}

void PE1MEW_TTNMapperNode::gpsDataReceived(void){
  _lastGPSDataTime = millis();
}

void PE1MEW_TTNMapperNode::gpsFix(void){
  _lastGPSFixTime = millis();
}

void PE1MEW_TTNMapperNode::buildPacket(void){
  _latitudeBinary = ((_gps.location.lat() + 90) / 180.0) * 16777215;
  _longitudeBinary = ((_gps.location.lng() + 180) / 360.0) * 16777215;

  _txBuffer[0] = ( _latitudeBinary >> 16 ) & 0xFF;
  _txBuffer[1] = ( _latitudeBinary >> 8 ) & 0xFF;
  _txBuffer[2] =   _latitudeBinary & 0xFF;

  _txBuffer[3] = ( _longitudeBinary >> 16 ) & 0xFF;
  _txBuffer[4] = ( _longitudeBinary >> 8 ) & 0xFF;
  _txBuffer[5] =   _longitudeBinary & 0xFF;
 
  _altitudeGps =   _gps.altitude.meters();
  _txBuffer[6] = ( _altitudeGps >> 8 ) & 0xFF;
  _txBuffer[7] =   _altitudeGps & 0xFF;

  _hdopGps =       _gps.hdop.value()/10;
  _txBuffer[8] =   _hdopGps & 0xFF;
}

void PE1MEW_TTNMapperNode::initializeRadio(){
   unsigned long _startTime = 0;
  _ledAct.startBlink();
  
//  delay(100); //wait 100 ms for the RN2xx3's startup message
//  Serial1.flush();

  _lora->autobaud();

  //print out the HWEUI so that we can register it via ttnctl
  String hweui = _lora->hweui();
  
  while(hweui.length() != 16){
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the node.");

    // timeout and led indication
    _startTime = millis();
    while (millis() - _startTime < 10000){
      _ledAct.process();
    }
    
    hweui = _lora->hweui();
  }

//  while(hweui.length() != 16){
//  {
//    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
//    Serial.println(hweui);
//    digitalWrite(RN_RESET, LOW);
//    delay(1000);
//    digitalWrite(RN_RESET, HIGH);
//    delay(1000);
//    hweui = myLora.hweui();
//  }

  Serial.println("When using OTAA, register this DevEUI: ");
  Serial.println(hweui);
  Serial.println("RN2xx3 firmware version:");
  Serial.println(_lora->sysver());

  Serial.println("Setting frequency plan");
  _lora->setFrequencyPlan(TTN_EU_DRIVETEST);   //configure your keys and join the network

  
  bool join_result = false;

#if defined(ABP)
  Serial.println("Mode: ABP");
  join_result = _lora->initABP(devAddr, nwkSKey, appSKey); // unit 3
#else
  Serial.println("Mode: OTAA");
  join_result = _lora->initOTAA(appEUI, appKEY, devEUI); // unit 3
#endif

  while(!join_result){
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    
    // timeout and led indication
    _startTime = millis();
    while (millis() - _startTime < 60000){ // Wait 1 minute before retrying
      _ledAct.process();
    }
    join_result = _lora->init();
  }
  Serial.println("Successfully joined");
  delay(1000); // wait a second before continuing.

  _lora->setDR(DEFAULT_DR);
  
  _ledAct.setOff();
}

bool PE1MEW_TTNMapperNode::testGeoFence(void){
  bool returnValue = false;
  double distance = 0;

  for (int i = 0; i < COORDINATES_COUNT; i++){
    distance = _gps.distanceBetween( _gps.location.lat(),
                                     _gps.location.lng(),
                                     coordinates[i][0],
                                     coordinates[i][1] );

    if (distance < GEOFENCE_DIAMETER){
      returnValue = true;
    }
  }
 
  return returnValue;
}

bool PE1MEW_TTNMapperNode::evaluateMoving(void){
  double distance = 0.0;
  double averageDistance = 0.0;
  
  // Calculate distance to last known coordinates
  distance = _gps.distanceBetween( _gps.location.lat(),
                                   _gps.location.lng(),
                                   _lastCoordinate.latitude,
                                   _lastCoordinate.longitude );
  
  // add distance to average buffer 
  _averageDistanceBuffer.addValue((float)distance);
  averageDistance = _averageDistanceBuffer.getAverage();
  
  // evaluate average distance
  // test for average distance and that the last distance was not one while moving.
  if ((averageDistance < MOVING_TEST_DISTANCE) && (distance < MOVING_TEST_DISTANCE)){
    _isMoving = false;
  }else{
    _isMoving = true;
  }
  
  // update current coordinates for next usage
  _lastCoordinate.latitude = _gps.location.lat();
  _lastCoordinate.longitude = _gps.location.lng();
  
  return _isMoving;
}

bool PE1MEW_TTNMapperNode::evaluateMoving(double distanceMeters){
  double distance = 0.0;
//  double averageDistance = 0.0;
  
  // Calculate distance to last known coordinates
  distance = _gps.distanceBetween( _gps.location.lat(),
                                   _gps.location.lng(),
                                   _lastCoordinate.latitude,
                                   _lastCoordinate.longitude );
  
  // test for distance travelled while moving.
  if (distance < distanceMeters){
    _isMoving = false;
  }else{
    _isMoving = true;
  }
  
  // update current coordinates for next usage
  _lastCoordinate.latitude  = _gps.location.lat();
  _lastCoordinate.longitude = _gps.location.lng();
  
  return _isMoving;
}
