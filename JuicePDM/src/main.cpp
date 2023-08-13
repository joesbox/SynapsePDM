#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Bounce2.h>
#include <Globals.h>
#include <OutputHandler.h>

void setup()
{
  Serial.begin(9600);
  InititalizeData();
  Channels[1].ChanType = DIG_ACT_HIGH_PWM;
  Channels[1].Enabled = true;
  Channels[1].PWMSetDuty = 255;
  Run();
}

void loop()
{
  if (task1 >= 1000)
  {
    task1 = 0;
    Serial.print("Channel 1 analog: ");
    Serial.println(CPU_TICK_MICROS, 32);
  }
}