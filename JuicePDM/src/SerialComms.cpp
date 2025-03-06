/*  SerialComms.cpp Serial comms variables, functions and data handling.
    Copyright (c) 2023 Joe Mann.  All right reserved.

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
#include "SerialComms.h"

ChannelConfigUnion SerialChannelData;
CRC32 crcSerial;

void InitialiseSerial()
{
    Serial.begin(115200);
#ifdef DEBUG
    while(!Serial)
    {
        // Wait for connectio
    }
#endif
}

void SleepComms()
{
    Serial.end();
}

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
        uint32_t checkSum = 0;
        byte float2Byte[4];
        byte chanSize = 0;
        byte send = 0;

        send = SERIAL_HEADER & 255;
        Serial.write(send);
        checkSum += send;

        send = SERIAL_HEADER >> 8;
        Serial.write(send);
        checkSum += send;

        Serial.write(COMMAND_ID_REQUEST);
        checkSum += COMMAND_ID_REQUEST;

        Serial.write(NUM_CHANNELS);
        checkSum += NUM_CHANNELS;

        chanSize = sizeof(Channels) / NUM_CHANNELS;

        for (int i = 0; i < NUM_CHANNELS; i++)
        {
            Serial.write((byte)Channels[i].ChanType);
            checkSum += (byte)Channels[i].ChanType;

            memcpy(&float2Byte, &Channels[i].CurrentLimitHigh, sizeof(Channels[i].CurrentLimitHigh));
            for (uint j = 0; j < sizeof(float2Byte); j++)
            {
                Serial.write(float2Byte[j]);
                checkSum += float2Byte[j];
            }

            Serial.write(Channels[i].CurrentSensePin);
            checkSum += Channels[i].CurrentSensePin;

            memcpy(&float2Byte, &Channels[i].CurrentThresholdHigh, sizeof(Channels[i].CurrentThresholdHigh));
            for (uint j = 0; j < sizeof(float2Byte); j++)
            {
                Serial.write(float2Byte[j]);
                checkSum += float2Byte[j];
            }

            memcpy(&float2Byte, &Channels[i].CurrentThresholdLow, sizeof(Channels[i].CurrentThresholdLow));
            for (uint j = 0; j < sizeof(float2Byte); j++)
            {
                Serial.write(float2Byte[j]);
                checkSum += float2Byte[j];
            }

            memcpy(&float2Byte, &Channels[i].CurrentValue, sizeof(Channels[i].CurrentValue));
            for (uint j = 0; j < sizeof(float2Byte); j++)
            {
                Serial.write(float2Byte[j]);
                checkSum += float2Byte[j];
            }

            Serial.write(Channels[i].Enabled);
            checkSum += Channels[i].Enabled;

            Serial.write(Channels[i].ErrorFlags);
            checkSum += Channels[i].ErrorFlags;

            Serial.write(Channels[i].GroupNumber);
            checkSum += Channels[i].GroupNumber;

            Serial.write(Channels[i].InputControlPin);
            checkSum += Channels[i].InputControlPin;

            Serial.write(Channels[i].MultiChannel);
            checkSum += Channels[i].MultiChannel;

            Serial.write(Channels[i].PWMSetDuty);
            checkSum += Channels[i].PWMSetDuty;

            Serial.write(Channels[i].Retry);
            checkSum += Channels[i].Retry;

            Serial.write(Channels[i].RetryCount);
            checkSum += Channels[i].RetryCount;

            memcpy(&float2Byte, &Channels[i].RetryDelay, sizeof(Channels[i].RetryDelay));
            for (uint j = 0; j < sizeof(float2Byte); j++)
            {
                Serial.write(float2Byte[j]);
                checkSum += float2Byte[j];
            }
        }

        send = SERIAL_TRAILER & 255;
        checkSum += send;
        Serial.write(send);

        send = SERIAL_TRAILER >> 8;
        checkSum += send;
        Serial.write(send);

        send = checkSum & 0xFF;
        send = 0xFF - send;
        Serial.write(send);

        break;
    }
    case COMMAND_ID_NEWCONFIG:
    {
        unsigned int i = 0;
        uint32_t recvChecksum = 0;
        while (!Serial.available())
        {
            if (i <= sizeof(SystemConfigUnion))
            {
                SerialChannelData.dataBytes[i] = Serial.read();
                i++;
            }
            else
            {
                recvChecksum <<= 8;
                recvChecksum |= Serial.read();
            }
        }

        // Calculate stored config bytes CRC
        uint32_t checksum = CRC32::calculate(SerialChannelData.dataBytes, sizeof(ChannelConfigUnion));

        // Respond with pass or fail. Copy the new config over if the checksum has passed.
        if (checksum == recvChecksum)
        {
            // Copy relevant channel data
            for (int j = 0; j < NUM_CHANNELS; j++)
            {
                Channels[j].ChanType = SerialChannelData.data[j].ChanType;
                Channels[j].OutputControlPin = SerialChannelData.data[j].OutputControlPin;
                Channels[j].CurrentLimitHigh = SerialChannelData.data[j].CurrentLimitHigh;
                Channels[j].CurrentSensePin = SerialChannelData.data[j].CurrentSensePin;
                Channels[j].CurrentThresholdHigh = SerialChannelData.data[j].CurrentThresholdHigh;
                Channels[j].CurrentThresholdLow = SerialChannelData.data[j].CurrentThresholdLow;
                Channels[j].Enabled = SerialChannelData.data[j].Enabled;
                Channels[j].GroupNumber = SerialChannelData.data[j].GroupNumber;
                Channels[j].InputControlPin = SerialChannelData.data[j].InputControlPin;
                Channels[j].PWMSetDuty = SerialChannelData.data[j].PWMSetDuty;
                Channels[j].Retry = SerialChannelData.data[j].Retry;
                Channels[j].RetryCount = SerialChannelData.data[j].RetryCount;
                Channels[j].RetryDelay = SerialChannelData.data[j].RetryDelay;
            }

            // Set the system parameters
            //SystemParams.CANResEnabled = SerialConfigData.data.sysParams.CANResEnabled;

            Serial.write(COMMAND_ID_CONFIM);
        }
        else
        {
            Serial.write(COMMAND_ID_CHECKSUM_FAIL);
        }
    }
    break;

    case COMMAND_ID_SAVECHANGES:
        SaveChannelConfig();
        break;

    case COMMAND_ID_FW_VER:
        Serial.write(FW_VER);
        break;

    case COMMAND_ID_BUILD_DATE:
        Serial.write(BUILD_DATE);
        break;
    }
}