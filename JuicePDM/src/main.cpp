#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Bounce2.h>
#include <Globals.h>
#include <OutputHandler.h>

OutputHandler outHandler(NUM_CHANNELS);

void setup()
{
  Serial.begin(9600);
  outHandler.Initialize();
  delay(10);
  
}

void loop()
{
  if (task1 >= 1000)
  {
    task1 = 0;
    Serial.print("Channel 1 name: ");
  Serial.println(outHandler.Channels[0].ChannelName);
  }
}