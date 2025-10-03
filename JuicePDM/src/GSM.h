/*  GSM.h GSM and GPS variables, functions and data handling.
    Copyright (c) 2025 Joe Mann.  All right reserved.

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

*/

#ifndef GSM_H
#define GSM_H

#include <Arduino.h>
#include <Globals.h>

enum SIM7600Commands
{
  GPS,              // Retrieve GPS data
  HTTP,             // Perform HTTP GET request
  SMS,              // Send SMS
  MQTT,             // MQTT commands
  MQTT_PUBLISH,     // Publish MQTT message
  MQTT_SUBSCRIBE,   // Subscribe to MQTT topic
  MQTT_UNSUBSCRIBE, // Unsubscribe from MQTT topic
  MQTT_CONNECT,     // Connect to MQTT server
  MQTT_DISCONNECT,  // Disconnect from MQTT server
  MQTT_PING,        // Ping MQTT server
  MQTT_STATUS,      // Get MQTT status
};

/// @brief Initialise GSM/GPS
void InitialiseGSM(bool enableData);

/// @brief Trigger SIM7600 module update
/// @param command Command to execute
void UpdateSIM7600(SIM7600Commands command);

/// @brief Parse GPS response data
/// @param response Response data
void parseGPSData(const char *response);

/// @brief GPS fix flag
extern bool GPSFix;

/// @brief Data connected flag
extern bool dataConnected;

/// @brief GPS Latitude
extern float lat;

/// @brief GPS Longitude
extern float lon;

/// @brief GPS Speed (default is knots)
extern float speed;

/// @brief GPS Altitude (default is metres);
extern float alt;

/// @brief GPS Accuracy
extern float accuracy;

/// @brief GPS Visible satellites
extern int vsat;

/// @brief GPS Used satellites
extern int usat;

/// @brief GPS Year
extern int year;

/// @brief GPS Month
extern int month;

/// @brief GPS Day
extern int day;

/// @brief GPS Hour
extern int hour;

/// @brief GPS Minute
extern int minute;

/// @brief GPS Second
extern int second;

#endif // GSM_H