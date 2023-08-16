/*  JuicePDM - CAN enabled Power Distribution Module with 6 channels.

    Code herin specifically applies to the application of Infineon BTS50010 High-Side Drivers
    
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
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE,
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

    Version history:
    2023-08-14        v0.1.0        Initial beta
*/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Bounce2.h>
#include <Globals.h>
#include <OutputHandler.h>
#include <InputHandler.h>
#include <Storage.h>

void setup()
{
  Serial.begin(9600);
  InititalizeData();
  Channels[1].ChanType = DIG_ACT_HIGH_PWM;
  Channels[1].Enabled = true;
  Channels[1].PWMSetDuty = 10;
  HandleOutputs();
  SaveConfig();
  Serial.println(ConfigData.data.channelConfigStored[0].ControlPin);
}

void loop()
{
  if (task1 >= 1000)
  {
    task1 = 0;
   // Serial.print("Channel 1 analog: ");
   // Serial.println(Channels[1].AnalogRaw);
   Serial.println(ConfigData.data.channelConfigStored[0].ControlPin);
  }
  
  if (task2 >= TASK_2_INTERVAL)
  {
    // Read input channel status
    HandleInputs();    
    
  }
}