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

void InitialiseSerial()
{
    Serial.begin(921600, SERIAL_8E2); // 921600 baud, 8 data bits, even parity, 2 stop bits

#ifdef DEBUG
    while (!Serial)
    {
        // Wait for connection
    }
#endif
}

void SleepComms()
{
    Serial.end();
}

void CheckSerial()
{
    if (Serial.available())
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
            uint32_t checkSum = 0;
            byte fourBytePacket[4];
            byte twoBytePacket[2];
            byte chanSize = 0;
            byte send = 0;

            send = SERIAL_HEADER & 0XFF;
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

                memcpy(&fourBytePacket, &Channels[i].CurrentLimitHigh, sizeof(Channels[i].CurrentLimitHigh));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    Serial.write(fourBytePacket[j]);
                    checkSum += fourBytePacket[j];
                }

                Serial.write(Channels[i].CurrentSensePin);
                checkSum += Channels[i].CurrentSensePin;

                memcpy(&fourBytePacket, &Channels[i].CurrentThresholdHigh, sizeof(Channels[i].CurrentThresholdHigh));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    Serial.write(fourBytePacket[j]);
                    checkSum += fourBytePacket[j];
                }

                memcpy(&fourBytePacket, &Channels[i].CurrentThresholdLow, sizeof(Channels[i].CurrentThresholdLow));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    Serial.write(fourBytePacket[j]);
                    checkSum += fourBytePacket[j];
                }

                memcpy(&fourBytePacket, &Channels[i].CurrentValue, sizeof(Channels[i].CurrentValue));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    Serial.write(fourBytePacket[j]);
                    checkSum += fourBytePacket[j];
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

                Serial.write(Channels[i].Retry);
                checkSum += Channels[i].Retry;

                Serial.write(Channels[i].RetryCount);
                checkSum += Channels[i].RetryCount;

                memcpy(&fourBytePacket, &Channels[i].InrushDelay, sizeof(Channels[i].InrushDelay));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    Serial.write(fourBytePacket[j]);
                    checkSum += fourBytePacket[j];
                }
            }

            Serial.write(NUM_ANA_CHANNELS);
            checkSum += NUM_ANA_CHANNELS;

            for (int i = 0; i < NUM_ANA_CHANNELS; i++)
            {
                Serial.write(AnalogueIns[i].PullUpEnable);
                checkSum += AnalogueIns[i].PullUpEnable;
                Serial.write(AnalogueIns[i].PullDownEnable);
                checkSum += AnalogueIns[i].PullDownEnable;
                Serial.write(AnalogueIns[i].IsDigital);
                checkSum += AnalogueIns[i].IsDigital;
                memcpy(&fourBytePacket, &AnalogueIns[i].OnThreshold, sizeof(AnalogueIns[i].OnThreshold));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    Serial.write(fourBytePacket[j]);
                    checkSum += fourBytePacket[j];
                }

                memcpy(&fourBytePacket, &AnalogueIns[i].OffThreshold, sizeof(AnalogueIns[i].OffThreshold));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    Serial.write(fourBytePacket[j]);
                    checkSum += fourBytePacket[j];
                }

                memcpy(&fourBytePacket, &AnalogueIns[i].ScaleMin, sizeof(AnalogueIns[i].ScaleMin));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    Serial.write(fourBytePacket[j]);
                    checkSum += fourBytePacket[j];
                }

                memcpy(&fourBytePacket, &AnalogueIns[i].ScaleMax, sizeof(AnalogueIns[i].ScaleMax));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    Serial.write(fourBytePacket[j]);
                    checkSum += fourBytePacket[j];
                }

                Serial.write(AnalogueIns[i].PWMMin);
                checkSum += AnalogueIns[i].PWMMin;

                Serial.write(AnalogueIns[i].PWMMax);
                checkSum += AnalogueIns[i].PWMMax;
            }

            // Send system parameters
            memcpy(&fourBytePacket, &SystemParams.SystemTemperature, sizeof(SystemParams.SystemTemperature));
            for (uint j = 0; j < sizeof(fourBytePacket); j++)
            {
                Serial.write(fourBytePacket[j]);
                checkSum += fourBytePacket[j];
            }

            Serial.write(SystemParams.CANResEnabled);
            checkSum += SystemParams.CANResEnabled;

            memcpy(&fourBytePacket, &SystemParams.VBatt, sizeof(SystemParams.VBatt));
            for (uint j = 0; j < sizeof(fourBytePacket); j++)
            {
                Serial.write(fourBytePacket[j]);
                checkSum += fourBytePacket[j];
            }

            memcpy(&fourBytePacket, &SystemParams.SystemCurrent, sizeof(SystemParams.SystemCurrent));
            for (uint j = 0; j < sizeof(fourBytePacket); j++)
            {
                Serial.write(fourBytePacket[j]);
                checkSum += fourBytePacket[j];
            }

            Serial.write(SystemParams.SystemCurrentLimit);
            checkSum += SystemParams.SystemCurrentLimit;

            memcpy(&twoBytePacket, &SystemParams.ErrorFlags, sizeof(SystemParams.ErrorFlags));
            for (uint j = 0; j < sizeof(twoBytePacket); j++)
            {
                Serial.write(twoBytePacket[j]);
                checkSum += twoBytePacket[j];
            }

            memcpy(&twoBytePacket, &SystemParams.ChannelDataCANID, sizeof(SystemParams.ChannelDataCANID));
            for (uint j = 0; j < sizeof(twoBytePacket); j++)
            {
                Serial.write(twoBytePacket[j]);
                checkSum += twoBytePacket[j];
            }

            memcpy(&twoBytePacket, &SystemParams.SystemDataCANID, sizeof(SystemParams.SystemDataCANID));
            for (uint j = 0; j < sizeof(twoBytePacket); j++)
            {
                Serial.write(twoBytePacket[j]);
                checkSum += twoBytePacket[j];
            }

            memcpy(&twoBytePacket, &SystemParams.ConfigDataCANID, sizeof(SystemParams.ConfigDataCANID));
            for (uint j = 0; j < sizeof(twoBytePacket); j++)
            {
                Serial.write(twoBytePacket[j]);
                checkSum += twoBytePacket[j];
            }

            memcpy(&fourBytePacket, &SystemParams.IMUwakeWindow, sizeof(SystemParams.IMUwakeWindow));
            for (uint j = 0; j < sizeof(fourBytePacket); j++)
            {
                Serial.write(fourBytePacket[j]);
                checkSum += fourBytePacket[j];
            }

            Serial.write(SystemParams.SpeedUnitPref);
            checkSum += SystemParams.SpeedUnitPref;

            Serial.write(SystemParams.DistanceUnitPref);
            checkSum += SystemParams.DistanceUnitPref;

            Serial.write(SystemParams.AllowData);
            checkSum += SystemParams.AllowData;

            Serial.write(SystemParams.AllowGPS);
            checkSum += SystemParams.AllowGPS;

            memcpy(&fourBytePacket, &SOC, sizeof(SOC));
            for (uint j = 0; j < sizeof(fourBytePacket); j++)
            {
                Serial.write(fourBytePacket[j]);
                checkSum += fourBytePacket[j];
            }

            memcpy(&fourBytePacket, &SOH, sizeof(SOH));
            for (uint j = 0; j < sizeof(fourBytePacket); j++)
            {
                Serial.write(fourBytePacket[j]);
                checkSum += fourBytePacket[j];
            }

            send = SERIAL_TRAILER & 0xFF;
            checkSum += send;
            Serial.write(send);

            send = SERIAL_TRAILER >> 8;
            checkSum += send;
            Serial.write(send);

            // Send the checksum
            Serial.write((uint8_t *)&checkSum, sizeof(checkSum));

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
                    Channels[j].Retry = SerialChannelData.data[j].Retry;
                    Channels[j].RetryCount = SerialChannelData.data[j].RetryCount;
                    Channels[j].InrushDelay = SerialChannelData.data[j].InrushDelay;

                    // Limit checking
                    if (Channels[j].CurrentLimitHigh > CURRENT_MAX)
                    {
                        Channels[j].CurrentLimitHigh = CURRENT_MAX;
                    }
                    if (Channels[j].CurrentThresholdHigh > CURRENT_MAX)
                    {
                        Channels[j].CurrentThresholdHigh = CURRENT_MAX;
                    }
                    if (Channels[j].CurrentThresholdLow < 0.0)
                    {
                        Channels[j].CurrentThresholdLow = 0.0;
                    }
                    if (Channels[j].InrushDelay > INRUSH_MAX)
                    {
                        Channels[j].InrushDelay = INRUSH_MAX;
                    }
                    if (Channels[j].InrushDelay < 0.0)
                    {
                        Channels[j].InrushDelay = 0.0;
                    }
                }

                // Set the system parameters
                // SystemParams.CANResEnabled = SerialConfigData.data.sysParams.CANResEnabled;

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
}