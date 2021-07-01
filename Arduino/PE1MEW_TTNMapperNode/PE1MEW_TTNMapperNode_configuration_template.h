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

/// \file PE1MEW_TTNMapperNode_configuration.h
/// \brief TTN Mapper configuration parameters
/// \date 18-09-2018
/// \author Remko Welling (PE1MEW)
/// \version 1.0  Initial version

/// \brief Various keys for personalisation of the TTN MApper node.

//#define unit_1  // Production in drivetest vehicle
#define unit_2

/// Set your DevAddr, NwkSKey, AppSKey
#if defined(unit_2) 
  const char devAddr[] = "260ABCDE";                           ///< Device address
#elif defined(unit_1)
  const char devAddr[] = "26012345";                           ///< Device address
#else
  #error "No drivetest_unit configured."
#endif
  const char nwkSKey[] = "00000000000000000000000000000000";   ///< Network Session key
  const char appSKey[] = "00000000000000000000000000000000";   ///< Application Session key

#define GEOFENCE_DIAMETER         300   ///< diameter of the geofence circle 
#define COORDINATES_COUNT         2     ///< Number of geofences to be evaluated

/// \brief Defines geofence coordinates
/// These coordinates are the center of a circle 
/// The number of coordinates shall be the same as the COORDINATES_COUNT define!
const double coordinates[3][2] = { 
  {52.00001, 5.10003},  ///< Coordinate 1: 
  {52.00001, 5.20007},  ///< Coordinate 2: 
  {52.00002, 5.90008}   ///< Coordinate 3: 
};

/// Brutal method to switch geofence on and off
/// \todo solve this.
//#define GEOFENCE_3  ///< Enable to test for geofence 3


#define MOVING_TEST_DISTANCE      20   ///< distance in meters that should be travelled to get moving state.
#define MOVING_COUNTER_MAXIMUM    4    ///< After this number of transmissions moving less than MOVING_TEST_DISTANCE meter, static operation is assumed.

