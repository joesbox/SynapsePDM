/*  SerialComms.cpp Serial comms variables, functions and data handling.
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
#include "SerialComms.h"

/// @brief Fired when begin command is recieved. Sends confirm command back.
void onSerialBegin()
{
    ssp.writeCommand(COMMAND_ID_CONFIM);
}

/// @brief Reads and stores the set of channel config data
void onReceivedValues()
{
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        byte channelType = ssp.readByte();
        switch (channelType)
        {
            case 0:
                Channels[i].ChanType = DIG;
            break;
            case 1:
                Channels[i].ChanType = DIG_PWM;
            break;
            case 2:
                Channels[i].ChanType = CAN_DIGITAL;
            break;
            case 3:
                Channels[i].ChanType = CAN_PWM;
            break;
        }

        Channels[i].CurrentLimitHigh = ssp.readFloat();
        Channels[i].CurrentThresholdHigh = ssp.readFloat();
        Channels[i].CurrentThresholdLow = ssp.readFloat();
        Channels[i].Enabled = ssp.readByte();
        Channels[i].MultiChannel = ssp.readByte();
        Channels[i].GroupNumber = ssp.readByte();
        Channels[i].PWMSetDuty = ssp.readByte();
        Channels[i].Retry = ssp.readByte();
        Channels[i].RetryCount = ssp.readByte();
        Channels[i].RetryDelay = ssp.readFloat();        
    }

    // Read end of transmission
     ssp.readEot(); 
}

/// @brief Initialise serial comms
void InitialiseSerialComms()
{
    ssp.init();
    ssp.registerCommand(COMMAND_ID_BEGIN, onSerialBegin);
    ssp.registerCommand(COMMAND_ID_RECEIVE, onReceivedValues);
}

/// @brief Check for incoming data
void CheckSerial()
{
    ssp.loop();
}

void onError(uint8_t errorNum)
{

}
