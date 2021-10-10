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

/// \file PE1MEW_Timer.h
/// \brief Simple expiry timer 
/// \date 18-09-2018
/// \author Remko Welling (PE1MEW)
/// \version 1.0  Initial version

#ifndef __PE1MEW_TIMER_H_
#define __PE1MEW_TIMER_H_

#include <stdint.h>

#include <arduino.h>  // millis()

/// \class PE1MEW_Timer
/// \brief Timer class
class PE1MEW_Timer
{
//variables
public:
protected:
private:

  /// end time at which timer shall expire
  unsigned long _msEndTime;

  /// State of the timer, true = running, false = expired.
  bool _active;
   
//functions
public:

	/// \brief default constructor.
	PE1MEW_Timer();

  /// \brief default destructor
	~PE1MEW_Timer();

  /// \brief Set timer
  /// \value msTime milliseconds in the future from now that the time shall expire
  /// \retval true = Timer ste successfully, false = Timer is running and not set.
  bool set(uint32_t msTime);

  /// \brief test if timer has expired.
  /// \retval true = Timer is expired, false = Timer is not expired.
  bool getExpired(void);
	
protected:
private:
}; // PE1MEW_Timer

#endif //__PE1MEW_TIMER_H_
