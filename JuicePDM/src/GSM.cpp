/*  GSM.cpp GSM and GPS variables, functions and data handling.
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

#include "GSM.h"

#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
#define SerialAT Serial1
#define TINY_GSM_DEBUG SerialMon

#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200

#define TINY_GSM_USE_GPRS true

const char apn[] = "three.co.uk";
const char gprsUser[] = "";
const char gprsPass[] = "";
float lat, lon, speed, alt, accuracy;
int vsat, usat, year, month, day, hour, minute, second;

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

void InitialiseGSM(bool enableData)
{
  pinMode(SIM_PWR, OUTPUT);
  pinMode(SIM_RST, OUTPUT);
  pinMode(SIM_FLIGHT, OUTPUT);

  // Power the module on
  digitalWrite(SIM_PWR, HIGH);

  SerialAT.begin(115200);
  modem.setNetworkMode(2);
  SerialAT.println("AT+CGPSNEMARATE=1");

  Serial.print("Modem init: ");
  Serial.println(modem.init());
  // delay(6000);
  // String modemInfo = modem.getModemInfo();
  // Serial.print("Modem Info: ");
  // Serial.println(modemInfo);
  Serial.print("Get SIM info: ");
  Serial.println(modem.getSimStatus());

  if (enableData)
  {
    // GPRS connection parameters are usually set after network registration
    Serial.print(F("Connecting to "));
    Serial.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass))
    {
      Serial.println(" fail");
      delay(10000);
      return;
    }
    Serial.println(" success");

    if (modem.isGprsConnected())
    {
      Serial.println("GPRS connected");
    }
  }

  // Now initialise GPS
  Serial.print("Enabling GPS: ");
  Serial.println(modem.enableGPS());
}

void UpdateGPS()
{

  // TODO: Change NEMA update
  bool success = modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy, &year, &month, &day, &hour, &minute, &second);

  if (success)
  {
    // Convert speed (knots) to preferred units
    switch (SystemParams.SpeedUnitPref)
    {
    case MPH:
      speed *= 1.15078F;
      break;

    case KPH:
      speed *= 1.852F;
      break;

    default:
      break;
    }

    // Convert distance to preferred units
    switch (SystemParams.DistanceUnitPref)
    {
    case Imperial:      
      alt *= 3.28084;
      accuracy += 3.28084;
      break;
    case Metric:
      // No coversion required
      break;

    default:
      break;
    }
  }
  digitalToggleFast(PA_15);
}
