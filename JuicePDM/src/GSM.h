/*  GSM.h GSM and GPS variables, functions and data handling.
    Copyright (c) 2025 Joe Mann.  All right reserved.

    This work is licensed under the Creative Commons
    Attribution-NonCommercial-ShareAlike 4.0 International License.
    To view a copy of this license, visit
    https://creativecommons.org/licenses/by-nc-sa/4.0/ or send a
    letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

    You are free to:
    - Share: Copy and redistribute the material in any medium or format.
    - Adapt: Remix, transform, and build upon the material.

    Under the following terms:
    - Attribution: You must give appropriate credit, provide a link to the license,
      and indicate if changes were made. You may do so in any reasonable manner,
      but not in any way that suggests the licensor endorses you or your use.
    - NonCommercial: You may not use the material for commercial purposes.
    - ShareAlike: If you remix, transform, or build upon the material,
      you must distribute your contributions under the same license as the original.

    DISCLAIMER: This software is provided "as is," without warranty of any kind,
    express or implied, including but not limited to the warranties of
    merchantability, fitness for a particular purpose, and noninfringement.
    In no event shall the authors or copyright holders be liable for any claim,
    damages, or other liability, whether in an action of contract, tort, or otherwise,
    arising from, out of, or in connection with the software or the use or
    other dealings in the software.
*/

#ifndef GSM_H
#define GSM_H

#include <Arduino.h>
#include <Globals.h>

/// @brief Initialise GSM/GPS
void InitialiseGSM(bool enableData);

/// @brief Update GPS info. Called periodically
void UpdateGPS();

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

/// @brief GPS Used satelites
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

#endif