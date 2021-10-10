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

/// \file PE1MEW_Button.cpp
/// \brief LED class to control a single switch connected to a pin on the Arduino board without use of delay() functions
/// \date 13-3-2017
/// \author Remko Welling (PE1MEW)
/// \version 1.0	Initial version

#include "PE1MEW_Button.h"

#include "Arduino.h"

// default constructor
PE1MEW_Button::PE1MEW_Button(uint8_t pin):
  _buttonPin(pin),
  _currentState(STATE_PRESSED),
  _nextState(STATE_PRESSED_T1),
  _testCounter(0),
  _pressStartTime(0)
{
  // initialize digital pin as an input.
  pinMode(_buttonPin, INPUT);
} //PE1MEW_Button

// default destructor
PE1MEW_Button::~PE1MEW_Button()
{
} //~PE1MEW_Button

eStateButton PE1MEW_Button::process(void)
{
  testButton();
  
  if (_testCounter == 0)
  {
    _nextState = STATE_NOT_PRESSED;
  }
  
  // pre process next state.
  // Make all required changes.
  if (_currentState != _nextState)
  {
    // Process new state (current)
    switch (_nextState)
    {
      case STATE_NOT_PRESSED:

      break;
        
      case STATE_PRESSED:

      break;
        
      case STATE_PRESSED_T1:

      break;

      default:

      break;
    }// end switch
  } // end if
  
	// switch button state to next state
	_currentState = _nextState;

  // Process current state activities.
  // This switch case is processing timeout timers
	switch (_currentState)
	{
		case STATE_NOT_PRESSED:
			if (_testCounter == PRESS_CONTER_MAX)
			{
				_nextState = STATE_PRESSED;
        _pressStartTime = millis();
			}
		break;
		
		case STATE_PRESSED:
      if (millis() - _pressStartTime > PRESS_TIME_T1)
      {
        _nextState = STATE_PRESSED_T1;
      }
    break;
    
    case STATE_PRESSED_T1:
      if (millis() - _pressStartTime > PRESS_TIME_T2)
      {
        _nextState = STATE_PRESSED_T2;
      }
    break;
    
    default:
			
		break;
	} // end switch
  return _currentState;
}

eStateButton PE1MEW_Button::getState(void)
{
  return _currentState;
}
 
void PE1MEW_Button::testButton(void)
{
     if (digitalRead(_buttonPin) == LOW)   // read the input pin
     {
         _testCounter++;
     }
     else
     {
        _testCounter--; 
     }
     
     if (_testCounter > PRESS_CONTER_MAX)
     {
        _testCounter = PRESS_CONTER_MAX;
     }
     if ( _testCounter < 0)
     {
        _testCounter = 0;
     }
 }

void PE1MEW_Button::reset(void)
{
  _nextState = STATE_NOT_PRESSED;
}

