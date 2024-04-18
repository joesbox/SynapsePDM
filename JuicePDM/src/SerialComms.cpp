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

ConfigUnion SerialConfigData;
CRC32 crcSerial;

void InitialiseSerial()
{
}

/// @brief Check for incoming data. Respond to a command byte or read all of the incoming config data and checksum.
void CheckSerial()
{
    byte nextByte = Serial.read();
    switch (nextByte)
    {
    case COMMAND_ID_BEGIN:
        Serial.write(COMMAND_ID_CONFIM);
        break;

    case COMMAND_ID_REQUEST:
    {
        // Send serial data packet and update CRC as we go
        crcSerial.reset();
        byte float2Byte[4];
        Serial.clear();
        Serial.write(SERIAL_HEADER & 255);
        crcSerial.update(SERIAL_HEADER & 255);

        Serial.write(SERIAL_HEADER >> 8);
        crcSerial.update(SERIAL_HEADER >> 8);

        Serial.write(COMMAND_ID_REQUEST);
        crcSerial.update(COMMAND_ID_REQUEST);

        Serial.write(NUM_CHANNELS);
        crcSerial.update(NUM_CHANNELS);

        for (int i = 0; i < NUM_CHANNELS; i++)
        {
            Serial.write((byte)Channels[i].ChanType);
            crcSerial.update((byte)Channels[i].ChanType);

            memcpy(&float2Byte, &Channels[i].CurrentLimitHigh, sizeof(Channels[i].CurrentLimitHigh));
            for (uint j = 0; j < sizeof(float2Byte); j++)
            {
                Serial.write(float2Byte[j]);
                crcSerial.update(float2Byte[j]);
            }

            Serial.write(Channels[i].CurrentSensePin);
            crcSerial.update(Channels[i].CurrentSensePin);

            memcpy(&float2Byte, &Channels[i].CurrentThresholdHigh, sizeof(Channels[i].CurrentThresholdHigh));
            for (uint j = 0; j < sizeof(float2Byte); j++)
            {
                Serial.write(float2Byte[j]);
                crcSerial.update(float2Byte[j]);
            }

            memcpy(&float2Byte, &Channels[i].CurrentThresholdLow, sizeof(Channels[i].CurrentThresholdLow));
            for (uint j = 0; j < sizeof(float2Byte); j++)
            {
                Serial.write(float2Byte[j]);
                crcSerial.update(float2Byte[j]);
            }

            memcpy(&float2Byte, &Channels[i].CurrentValue, sizeof(Channels[i].CurrentValue));
            for (uint j = 0; j < sizeof(float2Byte); j++)
            {
                Serial.write(float2Byte[j]);
                crcSerial.update(float2Byte[j]);
            }

            Serial.write(Channels[i].Enabled);
            crcSerial.update(Channels[i].Enabled);

            Serial.write(Channels[i].ErrorFlags);
            crcSerial.update(Channels[i].ErrorFlags);

            Serial.write(Channels[i].GroupNumber);
            crcSerial.update(Channels[i].GroupNumber);

            Serial.write(Channels[i].InputControlPin);
            crcSerial.update(Channels[i].InputControlPin);

            Serial.write(Channels[i].MultiChannel);
            crcSerial.update(Channels[i].MultiChannel);

            Serial.write(Channels[i].PWMSetDuty);
            crcSerial.update(Channels[i].PWMSetDuty);

            Serial.write(Channels[i].Retry);
            crcSerial.update(Channels[i].Retry);

            Serial.write(Channels[i].RetryCount);
            crcSerial.update(Channels[i].RetryCount);

            memcpy(&float2Byte, &Channels[i].RetryDelay, sizeof(Channels[i].RetryDelay));
            for (uint j = 0; j < sizeof(float2Byte); j++)
            {
                Serial.write(float2Byte[j]);
                crcSerial.update(float2Byte[j]);
            }
        }

        uint32_t crcVal = crcSerial.finalize();
        
        memcpy(&float2Byte, &crcVal, sizeof(crcVal));
        for (uint j = 0; j < sizeof(float2Byte); j++)
        {
            Serial.write(float2Byte[j]);
        }

        Serial.send_now();

        break;
    }
    case COMMAND_ID_NEWCONFIG:
    {
        unsigned int i = 0;
        uint32_t recvChecksum = 0;
        while (!Serial.available())
        {
            if (i <= sizeof(ConfigStruct))
            {
                SerialConfigData.dataBytes[i] = Serial.read();
                i++;
            }
            else
            {
                recvChecksum <<= 8;
                recvChecksum |= Serial.read();
            }
        }

        // Calculate stored config bytes CRC
        uint32_t checksum = CRC32::calculate(SerialConfigData.dataBytes, sizeof(ConfigStruct));

        // Respond with pass or fail. Copy the new config over if the checksum has passed.
        if (checksum == recvChecksum)
        {
            // Copy relevant channel data
            for (int j = 0; j < NUM_CHANNELS; j++)
            {
                Channels[j].ChanType = SerialConfigData.data.channelConfigStored[j].ChanType;
                Channels[j].ControlPin = SerialConfigData.data.channelConfigStored[j].ControlPin;
                Channels[j].CurrentLimitHigh = SerialConfigData.data.channelConfigStored[j].CurrentLimitHigh;
                Channels[j].CurrentSensePin = SerialConfigData.data.channelConfigStored[j].CurrentSensePin;
                Channels[j].CurrentThresholdHigh = SerialConfigData.data.channelConfigStored[j].CurrentThresholdHigh;
                Channels[j].CurrentThresholdLow = SerialConfigData.data.channelConfigStored[j].CurrentThresholdLow;
                Channels[j].Enabled = SerialConfigData.data.channelConfigStored[j].Enabled;
                Channels[j].GroupNumber = SerialConfigData.data.channelConfigStored[j].GroupNumber;
                Channels[j].InputControlPin = SerialConfigData.data.channelConfigStored[j].InputControlPin;
                Channels[j].PWMSetDuty = SerialConfigData.data.channelConfigStored[j].PWMSetDuty;
                Channels[j].Retry = SerialConfigData.data.channelConfigStored[j].Retry;
                Channels[j].RetryCount = SerialConfigData.data.channelConfigStored[j].RetryCount;
                Channels[j].RetryDelay = SerialConfigData.data.channelConfigStored[j].RetryDelay;
            }

            // Set the system parameters
            SystemParams.LEDBrightness = SerialConfigData.data.sysParams.LEDBrightness;
            SystemParams.CANResEnabled = SerialConfigData.data.sysParams.CANResEnabled;

            Serial.write(COMMAND_ID_CONFIM);
        }
        else
        {
            Serial.write(COMMAND_ID_CHECKSUM_FAIL);
        }
    }
    break;

    case COMMAND_ID_SAVECHANGES:
        SaveConfig();
        break;

    case COMMAND_ID_FW_VER:
        Serial.write(FW_VER);
        break;

    case COMMAND_ID_BUILD_DATE:
        Serial.write(BUILD_DATE);
        break;
    }
}