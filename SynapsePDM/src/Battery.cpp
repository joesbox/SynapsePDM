/*  Battery.cpp Battery charging, functions and data handling.
    Copyright (c) 2023 Joe Mann.  All right reserved.

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

#include <Battery.h>

int8_t batteryStatus;
int SOC;
int SOH;

void InitialiseBattery()
{
  pinMode(CHARGE_EN, OUTPUT);
  pinMode(BATT_INT, INPUT);

  digitalWrite(CHARGE_EN, LOW); // Active low

  batteryStatus = -1;
  if (lipo.begin())
  {
    batteryStatus = 0;
  }
  if (lipo.itporFlag()) // write config parameters only if needed
  {
    lipo.enterConfig();
    lipo.setCapacity(BATTERY_CAPACITY);
    lipo.setDesignEnergy(BATTERY_CAPACITY * 3.8f);
    lipo.setTerminateVoltage(TERMINATE_VOLTAGE);
    lipo.setTaperRate(10 * BATTERY_CAPACITY / TAPER_CURRENT);

    lipo.exitConfig();
  }
}

void ManageBattery()
{
  SOC = lipo.soc(); // Read state-of-charge (%)
  SOH = lipo.soh();  // Read state-of-health (%)

  if (SOC > 100)
  {
    SOC = 100; // Cap SOC at 100%
  }
  else if (SOC < 0)
  {
    SOC = 0; // Cap SOC at 0%
  }

  if (SOH > 100)
  {
    SOH = 100; // Cap SOH at 100%
  }
  else if (SOH < 0)
  {
    SOH = 0; // Cap SOH at 0%
  }
}

void printBatteryStats()
{
  // Read battery stats from the BQ27441-G1A
  unsigned int soc = SOC = lipo.soc();             // Read state-of-charge (%)
  unsigned int volts = lipo.voltage();             // Read battery voltage (mV)
  int current = lipo.current(AVG);                 // Read average current (mA)
  unsigned int fullCapacity = lipo.capacity(FULL); // Read full capacity (mAh)
  unsigned int capacity = lipo.capacity(REMAIN);   // Read remaining capacity (mAh)
  int power = lipo.power();                        // Read average power draw (mW)
  int health = lipo.soh();                         // Read state-of-health (%)

  // Now print out those values:
  String toPrint = String(soc) + "% | ";
  toPrint += String(volts) + " mV | ";
  toPrint += String(current) + " mA | ";
  toPrint += String(capacity) + " / ";
  toPrint += String(fullCapacity) + " mAh | ";
  toPrint += String(power) + " mW | ";
  toPrint += String(health) + "%";

  Serial.println(toPrint);
}