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

byte statusBuffer[1000] = {0};
int statusIndex = 0;

bool receivingConfig = false;

uint32_t lastComms = 0;

unsigned int readBufIdx = 0;

void InitialiseSerial()
{
    Serial.begin(921600); // 921600 baud. Doesn't matter on USB CDC. Good to match the PC side though.

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
    Serial1.end();
}

void CheckSerial()
{
    if ((millis() - lastComms > COMMS_TIMEOUT))
    {
        connectionStatus = 0; // Timeout - reset connection
        receivingConfig = false;
        readBufIdx = 0;
        recBytesRead = 0;
        for (int i = 0; i < NUM_CHANNELS; i++)
        {
            Channels[i].Override = false; // Clear any overrides
        }
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
            statusIndex = 0;    
            memset(statusBuffer, 0, sizeof(statusBuffer));        

            send = SERIAL_HEADER & 0XFF;
            statusBuffer[statusIndex++] = send;
            checkSum += send;

            send = SERIAL_HEADER >> 8;
            statusBuffer[statusIndex++] = send;
            checkSum += send;

            statusBuffer[statusIndex++] = COMMAND_ID_REQUEST;
            checkSum += COMMAND_ID_REQUEST;

            statusBuffer[statusIndex++] = NUM_CHANNELS;
            checkSum += NUM_CHANNELS;

            chanSize = sizeof(Channels) / NUM_CHANNELS;

            for (int i = 0; i < NUM_CHANNELS; i++)
            {
                statusBuffer[statusIndex++] = (byte)Channels[i].ChanType;
                checkSum += (byte)Channels[i].ChanType;

                statusBuffer[statusIndex++] = Channels[i].Override;
                checkSum += Channels[i].Override;

                statusBuffer[statusIndex++] = Channels[i].CurrentSensePin;
                checkSum += Channels[i].CurrentSensePin;

                memcpy(&fourBytePacket, &Channels[i].CurrentThresholdHigh, sizeof(Channels[i].CurrentThresholdHigh));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    statusBuffer[statusIndex++] = fourBytePacket[j];
                    checkSum += fourBytePacket[j];
                }

                memcpy(&fourBytePacket, &Channels[i].CurrentThresholdLow, sizeof(Channels[i].CurrentThresholdLow));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    statusBuffer[statusIndex++] = fourBytePacket[j];
                    checkSum += fourBytePacket[j];
                }

                memcpy(&fourBytePacket, &Channels[i].CurrentValue, sizeof(Channels[i].CurrentValue));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    statusBuffer[statusIndex++] = fourBytePacket[j];
                    checkSum += fourBytePacket[j];
                }

                statusBuffer[statusIndex++] = Channels[i].Enabled;
                checkSum += Channels[i].Enabled;

                statusBuffer[statusIndex++] = Channels[i].ErrorFlags;
                checkSum += Channels[i].ErrorFlags;

                statusBuffer[statusIndex++] = Channels[i].GroupNumber;
                checkSum += Channels[i].GroupNumber;

                statusBuffer[statusIndex++] = Channels[i].InputControlPin;
                checkSum += Channels[i].InputControlPin;

                statusBuffer[statusIndex++] = Channels[i].MultiChannel;
                checkSum += Channels[i].MultiChannel;

                statusBuffer[statusIndex++] = Channels[i].RetryCount;
                checkSum += Channels[i].RetryCount;

                memcpy(&fourBytePacket, &Channels[i].InrushDelay, sizeof(Channels[i].InrushDelay));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    statusBuffer[statusIndex++] = fourBytePacket[j];
                    checkSum += fourBytePacket[j];
                }

                memcpy(&threeBytePacket, &Channels[i].ChannelName, sizeof(Channels[i].ChannelName));
                for (uint j = 0; j < sizeof(threeBytePacket); j++)
                {
                    statusBuffer[statusIndex++] = threeBytePacket[j];
                    checkSum += threeBytePacket[j];
                }

                statusBuffer[statusIndex++] = Channels[i].RunOn;
                checkSum += Channels[i].RunOn;

                memcpy(&fourBytePacket, &Channels[i].RunOnTime, sizeof(Channels[i].RunOnTime));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    statusBuffer[statusIndex++] = fourBytePacket[j];
                    checkSum += fourBytePacket[j];
                }
            }

            statusBuffer[statusIndex++] = NUM_ANA_CHANNELS;
            checkSum += NUM_ANA_CHANNELS;

            for (int i = 0; i < NUM_ANA_CHANNELS; i++)
            {
                statusBuffer[statusIndex++] = AnalogueIns[i].PullUpEnable;
                checkSum += AnalogueIns[i].PullUpEnable;
                statusBuffer[statusIndex++] = AnalogueIns[i].PullDownEnable;
                checkSum += AnalogueIns[i].PullDownEnable;
                statusBuffer[statusIndex++] = AnalogueIns[i].IsDigital;
                checkSum += AnalogueIns[i].IsDigital;
                memcpy(&fourBytePacket, &AnalogueIns[i].OnThreshold, sizeof(AnalogueIns[i].OnThreshold));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    statusBuffer[statusIndex++] = fourBytePacket[j];
                    checkSum += fourBytePacket[j];
                }

                memcpy(&fourBytePacket, &AnalogueIns[i].OffThreshold, sizeof(AnalogueIns[i].OffThreshold));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    statusBuffer[statusIndex++] = fourBytePacket[j];
                    checkSum += fourBytePacket[j];
                }

                memcpy(&fourBytePacket, &AnalogueIns[i].ScaleMin, sizeof(AnalogueIns[i].ScaleMin));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    statusBuffer[statusIndex++] = fourBytePacket[j];
                    checkSum += fourBytePacket[j];
                }

                memcpy(&fourBytePacket, &AnalogueIns[i].ScaleMax, sizeof(AnalogueIns[i].ScaleMax));
                for (uint j = 0; j < sizeof(fourBytePacket); j++)
                {
                    statusBuffer[statusIndex++] = fourBytePacket[j];
                    checkSum += fourBytePacket[j];
                }

                statusBuffer[statusIndex++] = AnalogueIns[i].PWMMin;
                checkSum += AnalogueIns[i].PWMMin;

                statusBuffer[statusIndex++] = AnalogueIns[i].PWMMax;
                checkSum += AnalogueIns[i].PWMMax;
            }

            // Send system parameters
            memcpy(&fourBytePacket, &SystemParams.SystemTemperature, sizeof(SystemParams.SystemTemperature));
            for (uint j = 0; j < sizeof(fourBytePacket); j++)
            {
                statusBuffer[statusIndex++] = fourBytePacket[j];
                checkSum += fourBytePacket[j];
            }

            statusBuffer[statusIndex++] = SystemParams.CANResEnabled;
            checkSum += SystemParams.CANResEnabled;

            memcpy(&fourBytePacket, &SystemParams.VBatt, sizeof(SystemParams.VBatt));
            for (uint j = 0; j < sizeof(fourBytePacket); j++)
            {
                statusBuffer[statusIndex++] = fourBytePacket[j];
                checkSum += fourBytePacket[j];
            }

            memcpy(&fourBytePacket, &SystemParams.SystemCurrent, sizeof(SystemParams.SystemCurrent));
            for (uint j = 0; j < sizeof(fourBytePacket); j++)
            {
                statusBuffer[statusIndex++] = fourBytePacket[j];
                checkSum += fourBytePacket[j];
            }

            statusBuffer[statusIndex++] = SystemParams.SystemCurrentLimit;
            checkSum += SystemParams.SystemCurrentLimit;

            memcpy(&twoBytePacket, &SystemParams.ErrorFlags, sizeof(SystemParams.ErrorFlags));
            for (uint j = 0; j < sizeof(twoBytePacket); j++)
            {
                statusBuffer[statusIndex++] = twoBytePacket[j];
                checkSum += twoBytePacket[j];
            }

            memcpy(&twoBytePacket, &SystemParams.ChannelDataCANID, sizeof(SystemParams.ChannelDataCANID));
            for (uint j = 0; j < sizeof(twoBytePacket); j++)
            {
                statusBuffer[statusIndex++] = twoBytePacket[j];
                checkSum += twoBytePacket[j];
            }

            memcpy(&twoBytePacket, &SystemParams.SystemDataCANID, sizeof(SystemParams.SystemDataCANID));
            for (uint j = 0; j < sizeof(twoBytePacket); j++)
            {
                statusBuffer[statusIndex++] = twoBytePacket[j];
                checkSum += twoBytePacket[j];
            }

            memcpy(&twoBytePacket, &SystemParams.ConfigDataCANID, sizeof(SystemParams.ConfigDataCANID));
            for (uint j = 0; j < sizeof(twoBytePacket); j++)
            {
                statusBuffer[statusIndex++] = twoBytePacket[j];
                checkSum += twoBytePacket[j];
            }

            memcpy(&fourBytePacket, &SystemParams.IMUwakeWindow, sizeof(SystemParams.IMUwakeWindow));
            for (uint j = 0; j < sizeof(fourBytePacket); j++)
            {
                statusBuffer[statusIndex++] = fourBytePacket[j];
                checkSum += fourBytePacket[j];
            }

            statusBuffer[statusIndex++] = SystemParams.SpeedUnitPref;
            checkSum += SystemParams.SpeedUnitPref;

            statusBuffer[statusIndex++] = SystemParams.DistanceUnitPref;
            checkSum += SystemParams.DistanceUnitPref;

            statusBuffer[statusIndex++] = SystemParams.AllowData;
            checkSum += SystemParams.AllowData;

            statusBuffer[statusIndex++] = SystemParams.AllowGPS;
            checkSum += SystemParams.AllowGPS;

            send = SERIAL_TRAILER & 0xFF;
            checkSum += send;
            statusBuffer[statusIndex++] = send;

            send = SERIAL_TRAILER >> 8;
            checkSum += send;
            statusBuffer[statusIndex++] = send;

            // Send the checksum
            memcpy(&fourBytePacket, &checkSum, sizeof(checkSum));
            for (uint j = 0; j < sizeof(fourBytePacket); j++)
            {
                statusBuffer[statusIndex++] = fourBytePacket[j];
            }

            Serial.write(statusBuffer, statusIndex);

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
                        case 1: // Override flag
                            Channels[configBuffer[CONFIG_DATA_INDEX]].Override = configBuffer[CONFIG_DATA_START_INDEX];
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

                        default:
                            // Channel parameter out of range. Ignore packet
                            validPacket = false;
                            break;
                        }

                        break;
                    case CONFIG_DATA_ANALOGUE:
                        switch (configBuffer[CONFIG_PARAMETER_INDEX])
                        {
                        case 0: // Pull-up enable
                            AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].PullUpEnable = configBuffer[CONFIG_DATA_START_INDEX];
                            break;
                        case 1: // Pull-down enable
                            AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].PullDownEnable = configBuffer[CONFIG_DATA_START_INDEX];
                            break;
                        case 2: // Is digital
                            AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].IsDigital = configBuffer[CONFIG_DATA_START_INDEX];
                            break;
                        case 3: // Is threshold
                            AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].IsThreshold = configBuffer[CONFIG_DATA_START_INDEX];
                            break;
                        case 4: // On threshold
                            memcpy(&AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].OnThreshold, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].OnThreshold));
                            break;
                        case 5: // Off threshold
                            memcpy(&AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].OffThreshold, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].OffThreshold));
                            break;
                        case 6: // Scale min
                            memcpy(&AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].ScaleMin, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].ScaleMin));
                            break;
                        case 7: // Scale max
                            memcpy(&AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].ScaleMax, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].ScaleMax));
                            break;
                        case 8: // PWM min
                            AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].PWMMin = configBuffer[CONFIG_DATA_START_INDEX];
                            break;
                        case 9: // PWM max
                            AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].PWMMax = configBuffer[CONFIG_DATA_START_INDEX];
                            break;
                        default:
                            // Analogue parameter out of range. Ignore packet
                            validPacket = false;
                            break;
                        }

                        connectionStatus = 5;

                        break;

                    case CONFIG_DATA_SYSTEM:
                        connectionStatus = 7;

                        break;

                    case CONFIG_DATA_DIGITAL:
                        connectionStatus = 6;

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

            readBufIdx = 0;
            recBytesRead = 0;

            break;
        }

        case COMMAND_ID_SAVECHANGES:
        {
            SaveChannelConfig();
            SaveSystemConfig();
            SaveAnalogueConfig();

            bool allSaved = true;

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

            if (LoadAnalogueConfig())
            {
                allSaved &= true;
                InitialiseInputs();                
            }
            else
            {
                allSaved &= false;
                connectionStatus = 13;
            }

            if (allSaved)
            {
                backgroundDrawn = false; // Force display redraw
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
}