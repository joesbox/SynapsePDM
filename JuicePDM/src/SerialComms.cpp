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

ChannelConfigUnion SerialChannelData;
byte configBuffer[1000] = {0};

bool receivingConfig = false;

uint32_t lastComms = 0;

unsigned int readBufIdx = 0;

void InitialiseSerial()
{
    Serial.begin(9600, SERIAL_8E2); // 921600 baud, 8 data bits, even parity, 2 stop bits

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
    if (connectionStatus > 0 && (millis() - lastComms > COMMS_TIMEOUT))
    {
        connectionStatus = 0; // Timeout - reset connection
        receivingConfig = false;
        readBufIdx = 0;
        recBytesRead = 0;
    }
    if (Serial.available() && !receivingConfig)
    {
        lastComms = millis();
        byte nextByte = Serial.read();
        switch (nextByte)
        {
        case COMMAND_ID_BEGIN:
            Serial.write(COMMAND_ID_CONFIM);
            connectionStatus = 1;
            break;
        
        case COMMAND_ID_SKIP:
            Serial.write(COMMAND_ID_CONFIM);
            connectionStatus = 1;
            break;
            
        case COMMAND_ID_REQUEST:
        {
            // Send serial data packet and update CRC as we go
            uint32_t checkSum = 0;
            byte fourBytePacket[4];
            byte threeBytePacket[3];
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

                Serial.write(Channels[i].RetryCount);
                checkSum += Channels[i].RetryCount;

                memcpy(&fourBytePacket, &Channels[i].InrushDelay, sizeof(Channels[i].InrushDelay));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    Serial.write(fourBytePacket[j]);
                    checkSum += fourBytePacket[j];
                }

                memcpy(&threeBytePacket, &Channels[i].ChannelName, sizeof(Channels[i].ChannelName));
                for (uint j = 0; j < sizeof(threeBytePacket); j++)
                {
                    Serial.write(threeBytePacket[j]);
                    checkSum += threeBytePacket[j];
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
            connectionStatus = 2;
            // New config incoming, read all bytes into buffer
            uint32_t calcChecksum = 0;
            bool validPacket = false;
            delay(10); // Wait for data to start arriving

            while (Serial.available())
            {
                if (readBufIdx < 1000)
                {
                    connectionStatus = 3; // Receiving config data
                    configBuffer[readBufIdx] = Serial.read();
                    readBufIdx++;
                    recBytesRead = readBufIdx;
                }
                else
                {
                    Serial.read(); // Overflow - discard byte
                }
            }

            // Check header and trailer
            if ((configBuffer[0] == (SERIAL_HEADER & 0XFF)) && (configBuffer[1] == (SERIAL_HEADER >> 8)) &&
                (configBuffer[readBufIdx - 6] == (SERIAL_TRAILER & 0xFF)) && (configBuffer[readBufIdx - 5] == (SERIAL_TRAILER >> 8)))
            {
                connectionStatus = 3;
                validPacket = true;

                for (int i = 0; i < readBufIdx - 4; i++)
                {
                    calcChecksum += configBuffer[i];
                }

                uint32_t checksum = 0;
                checksum |= configBuffer[readBufIdx - 4];
                checksum |= configBuffer[readBufIdx - 3] << 8;
                checksum |= configBuffer[readBufIdx - 2] << 16;
                checksum |= configBuffer[readBufIdx - 1] << 24;

                // Checksum valid?
                if (checksum == calcChecksum)
                {
                    connectionStatus = 4;
                    validPacket = true;

                    // Copy the new config over
                    switch (configBuffer[CONFIG_TYPE_INDEX])
                    {
                    case CONFIG_DATA_CHANNELS:
                        connectionStatus = 4;
                        switch (configBuffer[CONFIG_PARAMETER_INDEX])
                        {
                        case 0: // Channel type
                            Channels[configBuffer[CONFIG_DATA_INDEX]].ChanType = (ChannelType)configBuffer[CONFIG_DATA_START_INDEX];
                            break;
                        case 1: // Current limit high
                            memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].CurrentLimitHigh, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].CurrentLimitHigh));
                            if (Channels[configBuffer[CONFIG_DATA_INDEX]].CurrentLimitHigh > CURRENT_MAX)
                            {
                                Channels[configBuffer[CONFIG_DATA_INDEX]].CurrentLimitHigh = CURRENT_MAX;
                            }
                            break;
                        case 2: // Current threshold high
                            memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].CurrentThresholdHigh, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].CurrentThresholdHigh));
                            if (Channels[configBuffer[CONFIG_DATA_INDEX]].CurrentThresholdHigh > CURRENT_MAX)
                            {
                                Channels[configBuffer[CONFIG_DATA_INDEX]].CurrentThresholdHigh = CURRENT_MAX;
                            }
                            break;
                        case 3: // Current threshold low
                            memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].CurrentThresholdLow, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].CurrentThresholdLow));
                            if (Channels[configBuffer[CONFIG_DATA_INDEX]].CurrentThresholdLow < 0.0)
                            {
                                Channels[configBuffer[CONFIG_DATA_INDEX]].CurrentThresholdLow = 0.0;
                            }
                            break;
                        case 4: // Enabled
                            Channels[configBuffer[CONFIG_DATA_INDEX]].Enabled = configBuffer[CONFIG_DATA_START_INDEX];
                            break;
                        case 5: // Group number
                            Channels[configBuffer[CONFIG_DATA_INDEX]].GroupNumber = configBuffer[CONFIG_DATA_START_INDEX];
                            break;
                        case 6: // Input control pin
                            Channels[configBuffer[CONFIG_DATA_INDEX]].InputControlPin = configBuffer[CONFIG_DATA_START_INDEX];
                            break;
                        case 7: // Multi channel
                            Channels[configBuffer[CONFIG_DATA_INDEX]].MultiChannel = configBuffer[CONFIG_DATA_START_INDEX];
                            break;
                        case 8: // Retry count
                            Channels[configBuffer[CONFIG_DATA_INDEX]].RetryCount = configBuffer[CONFIG_DATA_START_INDEX];
                            break;
                        case 9: // Inrush delay
                            memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].InrushDelay, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].InrushDelay));
                            if (Channels[configBuffer[CONFIG_DATA_INDEX]].InrushDelay > INRUSH_MAX)
                            {
                                Channels[configBuffer[CONFIG_DATA_INDEX]].InrushDelay = INRUSH_MAX;
                            }
                            if (Channels[configBuffer[CONFIG_DATA_INDEX]].InrushDelay < 0.0)
                            {
                                Channels[configBuffer[CONFIG_DATA_INDEX]].InrushDelay = 0.0;
                            }
                            break;
                        case 10: // Channel name
                            memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].ChannelName, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].ChannelName));
                            break;
                        case 11: // Run on
                            Channels[configBuffer[CONFIG_DATA_INDEX]].RunOn = configBuffer[CONFIG_DATA_START_INDEX];
                            break;
                        case 12: // Run on time
                            memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].RunOnTime, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].RunOnTime));
                            if (Channels[configBuffer[CONFIG_DATA_INDEX]].RunOnTime < 0)
                            {
                                Channels[configBuffer[CONFIG_DATA_INDEX]].RunOnTime = 0;
                            }
                            break;
                        case 13: // Name
                            memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].ChannelName, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].ChannelName));
                            break;

                        default:
                            // Channel parameter out of range. Ignore packet
                            validPacket = false;
                            break;
                        }

                        break;
                    case CONFIG_DATA_ANALOGUE:
                        connectionStatus = 5;

                        break;
                    case CONFIG_DATA_DIGITAL:
                        connectionStatus = 6;

                        break;
                    case CONFIG_DATA_SYSTEM:
                        connectionStatus = 7;

                        break;

                    default:
                        // Config type out of range. Ignore packet
                        validPacket = false;
                        break;
                    }
                }
                else // Checksum fail
                {
                    validPacket = false;
                    connectionStatus = 8;
                }
            }
            else // Header or trailer fail
            {
                validPacket = false;
                connectionStatus = 9;
            }

            if (validPacket)
            {
                Serial.write(COMMAND_ID_CONFIM);
                connectionStatus = 10;
            }
            else
            {
                Serial.write(COMMAND_ID_CHECKSUM_FAIL);                
            }
            
            break;
        }

        case COMMAND_ID_SAVECHANGES:
        {
            SaveChannelConfig();
            SaveSystemConfig();

            bool allSaved = true;

            /*allSaved &= LoadChannelConfig();
            allSaved &= LoadSystemConfig();
            allSaved &= LoadStorageConfig();*/

            if (LoadChannelConfig())
            {
                allSaved &= true;
            }
            else
            {
                allSaved &= false;
                connectionStatus = 11;            
            }

            if (LoadSystemConfig())
            {
                allSaved &= true;
            }
            else
            {
                allSaved &= false;
                connectionStatus = 12;            
            }

            Serial.write(allSaved ? COMMAND_ID_CONFIM
                                  : COMMAND_ID_CHECKSUM_FAIL);
            break;
        }
        case COMMAND_ID_FW_VER:
            Serial.write(FW_VER);
            break;

        case COMMAND_ID_BUILD_DATE:
            Serial.write(BUILD_DATE);
            break;
        }
    }
    else
    {
        /*if (receivingConfig)
        {
            uint32_t calcChecksum = 0;
            // Read incoming config data
            while (Serial.available())
            {
                if (readBufIdx < 1000)
                {
                    connectionStatus = 4; // Receiving config data
                    configBuffer[readBufIdx] = Serial.read();
                    readBufIdx++;
                    recBytesRead = readBufIdx;
                }
                else
                {
                    Serial.read(); // Overflow - discard byte
                }
            }

            // Sum the data to calculate checksum
            for (int i = 0; i < readBufIdx - 4; i++)
            {
                calcChecksum += configBuffer[i];
            }

            if (recBytesRead == NUM_CONFIG_BYTES)
            {
                connectionStatus = 5; // Config data received
                // Check header && trailer before calculating checksum
                if ((configBuffer[0] == (SERIAL_HEADER & 0XFF)) && (configBuffer[1] == (SERIAL_HEADER >> 8)) &&
                    (configBuffer[readBufIdx - 6] == (SERIAL_TRAILER & 0xFF)) && (configBuffer[readBufIdx - 5] == (SERIAL_TRAILER >> 8)))
                {
                    connectionStatus = 6; // Header and trailer valid
                    // Calculate stored config bytes CRC
                    uint32_t checksum = 0;
                    checksum |= configBuffer[readBufIdx - 3] & 0xFF;
                    checksum |= configBuffer[readBufIdx - 2] << 8 & 0xFF00;
                    checksum |= configBuffer[readBufIdx - 1] << 16 & 0xFF0000;
                    checksum |= configBuffer[readBufIdx] << 24 & 0xFF000000;

                    // Checksum valid?
                    if (checksum == calcChecksum)
                    {
                        connectionStatus = 7; // Checksum pass
                    }
                    else
                    {
                        Serial.write(COMMAND_ID_CHECKSUM_FAIL);
                        connectionStatus = 8; // Checksum fail
                    }
                    Serial.write(COMMAND_ID_CONFIM);
                }
                else
                {
                    Serial.write(COMMAND_ID_CHECKSUM_FAIL);
                }
            }
        }*/
    }
}