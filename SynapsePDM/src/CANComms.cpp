/*  CANComms.h CAN bus variables, functions and data handling.
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

#include "CANComms.h"

// Use CAN1 with ALT_2 pin configuration (PD0/PD1)
STM32_CAN Can(CAN1, ALT_2);

uint8_t aliveCounter = 0;

uint32_t EEPROMSaveTimout = 0;

bool pendingEEPROMSave = false;

bool saveEEPROMOnTimeout = false;

void InitialiseCAN()
{
    pinMode(CAN_BUS_RESISTOR_ENABLE, OUTPUT);
    if (SystemParams.CANResEnabled)
    {
        digitalWrite(CAN_BUS_RESISTOR_ENABLE, HIGH);
    }
    else
    {
        digitalWrite(CAN_BUS_RESISTOR_ENABLE, LOW);
    }

    // Initialize CAN bus
    Can.begin();

    // Set baud rate after begin()
    Can.setBaudRate(500000);

    // Accept all messages by default (bank 0 with mask 0 = accept all)
    Can.setFilterSingleMask(0, 0x000, 0x000, STD);
}

void ReadCANMessages()
{
    CAN_message_t msg;
    while (Can.read(msg))
    {
        if (msg.id == SystemParams.ChannelDataCANID)
        {
            // Channel status request. Reply on data CAN ID + 1 over 3 frames.
            for (int i = 0; i < NUM_CHANNELS; i++)
            {
                if (msg.buf[0] - 1 == i)
                {
                    CAN_message_t frame0;
                    frame0.id = SystemParams.ChannelDataCANID + 1;
                    frame0.len = 8;
                    frame0.flags.extended = 0;
                    frame0.flags.remote = 0;
                    frame0.buf[0] = 0; // Frame index
                    frame0.buf[1] = (uint8_t)(Channels[i].ChanType);
                    frame0.buf[2] = (ChannelRuntime[i].CurrentValue * 10);
                    frame0.buf[3] = Channels[i].Enabled;
                    uint16_t packedName = ((Channels[i].ChannelName[0] - 'A') << 10) | ((Channels[i].ChannelName[1] - 'A') << 5) | (Channels[i].ChannelName[2] - 'A');
                    frame0.buf[4] = (packedName >> 8) & 0xFF; // upper 8 bits
                    frame0.buf[5] = packedName & 0xFF;        // lower 8 bits
                    frame0.buf[6] = (Channels[i].CurrentThresholdLow * 10);
                    frame0.buf[7] = (Channels[i].CurrentThresholdHigh * 10);
                    Can.write(frame0);

                    CAN_message_t frame1;
                    frame1.id = SystemParams.ChannelDataCANID + 1;
                    frame1.len = 8;
                    frame1.flags.extended = 0;
                    frame1.flags.remote = 0;
                    frame1.buf[0] = 1; // Frame index
                    frame1.buf[1] = Channels[i].RetryCount;
                    frame1.buf[2] = Channels[i].InrushDelay >> 24 & 0xFF; // MSB
                    frame1.buf[3] = Channels[i].InrushDelay >> 16 & 0xFF;
                    frame1.buf[4] = Channels[i].InrushDelay >> 8 & 0xFF;
                    frame1.buf[5] = Channels[i].InrushDelay & 0xFF; // LSB
                    frame1.buf[6] = Channels[i].ActiveHigh;
                    frame1.buf[7] = Channels[i].RunOn;
                    Can.write(frame1);

                    CAN_message_t frame2;
                    frame2.id = SystemParams.ChannelDataCANID + 1;
                    frame2.len = 8;
                    frame2.flags.extended = 0;
                    frame2.flags.remote = 0;
                    frame2.buf[0] = 2;                                  // Frame index
                    frame2.buf[1] = Channels[i].RunOnTime >> 24 & 0xFF; // MSB
                    frame2.buf[2] = Channels[i].RunOnTime >> 16 & 0xFF;
                    frame2.buf[3] = Channels[i].RunOnTime >> 8 & 0xFF;
                    frame2.buf[4] = Channels[i].RunOnTime & 0xFF; // LSB
                    Can.write(frame2);
                }
            }
        }

        // Basic channel control message - F0
        if (msg.id == SystemParams.ChannelConfigDataCANID)
        {
            for (int i = 0; i < NUM_CHANNELS; i++)
            {
                if (msg.buf[0] - 1 == i)
                {
                    CANChannelEnableFlags[i] = (msg.buf[1] > 0) ? true : false;

                    if (Channels[i].ChanType == CAN_PWM)
                    {
                        if (msg.buf[1] <= 100)
                        {
                            Channels[i].PWMSetDuty = msg.buf[1];
                        }
                    }
                }
            }
        }

        // Channel config - F1
        if (msg.id == SystemParams.ChannelConfigDataCANID + 1)
        {
            for (int i = 0; i < NUM_CHANNELS; i++)
            {
                if (msg.buf[0] - 1 == i)
                {
                    // Dertermines which parameters should be set by this message
                    uint8_t paramMask = msg.buf[7];
                    // Channel type
                    if ((paramMask) & 0x01)
                    {
                        if (msg.buf[1] <= 5)
                        {
                            ChannelType newType;
                            switch (msg.buf[1])
                            {
                            case 0: // Digital
                                newType = DIG;
                                break;

                            case 1: // Digital PWM
                                newType = DIG_PWM;
                                break;

                            case 2: // Analogue threshold
                                newType = ANA;
                                break;

                            case 3: // Analogue scaled
                                newType = ANA_PWM;
                                break;

                            case 4: // CAN digital
                                newType = CAN_DIGITAL;
                                break;

                            case 5: // CAN PWM
                                newType = CAN_PWM;
                                break;
                            }
                            
                            if (newType != Channels[i].ChanType)
                            {
                                Channels[i].ChanType = newType;                                
                                pendingEEPROMSave = true;
                            }
                        }
                    }

                    // Channel name
                    if ((paramMask >> 1) & 0x01)
                    {
                        uint16_t packedName = ((uint16_t)msg.buf[2] << 8) | msg.buf[3];
                        char newName[4];
                        newName[0] = ((packedName >> 10) & 0x1F) + 'A';
                        newName[1] = ((packedName >> 5) & 0x1F) + 'A';
                        newName[2] = (packedName & 0x1F) + 'A';
                        newName[3] = 0;
                        
                        if (memcmp(newName, Channels[i].ChannelName, 3) != 0)
                        {
                            memcpy(Channels[i].ChannelName, newName, sizeof(Channels[i].ChannelName));                            
                            pendingEEPROMSave = true;
                        }
                    }

                    // Current threshold low
                    if ((paramMask >> 3) & 0x01)
                    {
                        if (msg.buf[4] <= MAX_CURRENT_X10)
                        {
                            uint8_t newThreshold = msg.buf[4] / 10;
                            if (newThreshold != Channels[i].CurrentThresholdLow)
                            {
                                Channels[i].CurrentThresholdLow = newThreshold;                                
                                pendingEEPROMSave = true;
                            }
                        }
                    }

                    // Current threshold high
                    if ((paramMask >> 4) & 0x01)
                    {
                        if (msg.buf[5] <= MAX_CURRENT_X10)
                        {
                            uint8_t newThreshold = msg.buf[5] / 10;
                            if (newThreshold != Channels[i].CurrentThresholdHigh)
                            {
                                Channels[i].CurrentThresholdHigh = newThreshold;                                
                                pendingEEPROMSave = true;
                            }
                        }
                    }

                    // Retry count
                    if ((paramMask >> 5) & 0x01)
                    {
                        if (msg.buf[6] != Channels[i].RetryCount)
                        {
                            Channels[i].RetryCount = msg.buf[6];                            
                            pendingEEPROMSave = true;
                        }
                    }
                }
            }
        }

        // Channel config - F2
        if (msg.id == SystemParams.ChannelConfigDataCANID + 2)
        {
            for (int i = 0; i < NUM_CHANNELS; i++)
            {
                if (msg.buf[0] - 1 == i)
                {
                    // Dertermines which parameters should be set by this message
                    uint8_t paramMask = msg.buf[7];

                    // Inrush delay
                    if ((paramMask >> 1) & 0x01)
                    {
                        uint32_t inrush = ((uint32_t)msg.buf[1] << 24) |
                                          ((uint32_t)msg.buf[2] << 16) |
                                          ((uint32_t)msg.buf[3] << 8) |
                                          (uint32_t)msg.buf[4];

                        if (inrush <= MAX_INRUSH_DELAY && inrush != Channels[i].InrushDelay)
                        {
                            Channels[i].InrushDelay = inrush;                            
                            pendingEEPROMSave = true;
                        }
                    }

                    // Active high
                    if ((paramMask >> 2) & 0x01)
                    {
                        bool newActiveHigh = (msg.buf[5] != 0);
                        if (newActiveHigh != Channels[i].ActiveHigh)
                        {
                            Channels[i].ActiveHigh = newActiveHigh;                            
                            pendingEEPROMSave = true;
                        }
                    }

                    // Run on
                    if ((paramMask >> 3) & 0x01)
                    {
                        bool newRunOn = (msg.buf[6] != 0);
                        if (newRunOn != Channels[i].RunOn)
                        {
                            Channels[i].RunOn = newRunOn;                            
                            pendingEEPROMSave = true;
                        }
                    }
                }
            }
        }

        // Channel config - F3
        if (msg.id == SystemParams.ChannelConfigDataCANID + 3)
        {
            for (int i = 0; i < NUM_CHANNELS; i++)
            {
                if (msg.buf[0] - 1 == i)
                {
                    // Dertermines which parameters should be set by this message
                    uint8_t paramMask = msg.buf[7];

                    if ((paramMask >> 1) & 0x01)
                    {
                        uint32_t runOn = ((uint32_t)msg.buf[1] << 24) |
                                         ((uint32_t)msg.buf[2] << 16) |
                                         ((uint32_t)msg.buf[3] << 8) |
                                         (uint32_t)msg.buf[4];

                        if (runOn <= MAX_RUN_ON_TIME && runOn != Channels[i].RunOnTime)
                        {
                            Channels[i].RunOnTime = runOn;                            
                            pendingEEPROMSave = true;
                        }
                    }
                }
            }
        }

        // System config message
        if (msg.id == SystemParams.SystemConfigDataCANID)
        {
            // Dertermines which parameters should be set by this message
            uint8_t paramMask = msg.buf[7];

            // System current limit
            if (paramMask & 0x01)
            {   
                if (msg.buf[0] <= SYSTEM_CURRENT_MAX && msg.buf[0] != SystemParams.SystemCurrentLimit)
                {
                    SystemParams.SystemCurrentLimit = msg.buf[0];
                    pendingEEPROMSave = true;
                }
            }

            // Speed unit preference
            if ((paramMask >> 1) & 0x01)
            {
                if (msg.buf[1] <= 1 && msg.buf[1] != SystemParams.SpeedUnitPref)
                {
                    SystemParams.SpeedUnitPref = msg.buf[1];
                    pendingEEPROMSave = true;
                }
            }

            // Distance unit preference
            if ((paramMask >> 2) & 0x01)
            {
                if (msg.buf[2] <= 1 && msg.buf[2] != SystemParams.DistanceUnitPref)
                {
                    SystemParams.DistanceUnitPref = msg.buf[2];                    
                    pendingEEPROMSave = true;
                }
            }

            // Allow data
            if ((paramMask >> 3) & 0x01)
            {
                if (msg.buf[3] <= 1 && msg.buf[3] != SystemParams.AllowData)
                {
                    SystemParams.AllowData = msg.buf[3];                    
                    pendingEEPROMSave = true;
                }
            }

            // Allow GPS
            if ((paramMask >> 4) & 0x01)
            {
                if (msg.buf[4] <= 1 && msg.buf[4] != SystemParams.AllowGPS)
                {
                    SystemParams.AllowGPS = msg.buf[4];                    
                    pendingEEPROMSave = true;
                }
            }

            // Allow motion detect
            if ((paramMask >> 5) & 0x01)
            {
                if (msg.buf[5] <= 1 && msg.buf[5] != SystemParams.AllowMotionDetect)
                {
                    SystemParams.AllowMotionDetect = msg.buf[5];                    
                    pendingEEPROMSave = true;
                }
            }

            // Motion dead time
            if ((paramMask >> 6) & 0x01)
            {
                if (msg.buf[6] <= MAX_MOTION_DEAD_TIME && msg.buf[6] != SystemParams.MotionDeadTime)
                {
                    SystemParams.MotionDeadTime = msg.buf[6];                    
                    pendingEEPROMSave = true;
                }
            }
        }

        if (pendingEEPROMSave)
        {
            pendingEEPROMSave = false;
            saveEEPROMOnTimeout = true;
            invalidateDisplay = true;
            EEPROMSaveTimout = millis() + EEPROM_WRITE_DELAY;
        }
    }
}

void BroadcastSystemStatus()
{
    // System status 1
    CAN_message_t systemStatusMsg1;
    systemStatusMsg1.id = SystemParams.SystemDataCANID;
    systemStatusMsg1.len = 8;
    systemStatusMsg1.flags.extended = 0;
    systemStatusMsg1.flags.remote = 0;
    systemStatusMsg1.buf[0] = aliveCounter++;
    systemStatusMsg1.buf[1] = SystemParams.SystemCurrentLimit;
    systemStatusMsg1.buf[2] = (uint8_t)SystemRuntimeParams.SystemTemperature;
    systemStatusMsg1.buf[3] = (uint8_t)(SystemRuntimeParams.VBatt * 10);
    uint16_t scaledCurrent = (uint16_t)(SystemRuntimeParams.SystemCurrent * 10);
    systemStatusMsg1.buf[4] = (scaledCurrent >> 8) & 0xFF;                             // MSB
    systemStatusMsg1.buf[5] = scaledCurrent & 0xFF;                                    // LSB
    systemStatusMsg1.buf[6] = (uint8_t)((SystemRuntimeParams.ErrorFlags >> 8) & 0xFF); // MSB
    systemStatusMsg1.buf[7] = (uint8_t)(SystemRuntimeParams.ErrorFlags & 0xFF);        // LSB
    Can.write(systemStatusMsg1);

    // System status 2
    CAN_message_t systemStatusMsg2;
    systemStatusMsg2.id = SystemParams.SystemDataCANID + 1;
    systemStatusMsg2.len = 8;
    systemStatusMsg2.flags.extended = 0;
    systemStatusMsg2.flags.remote = 0;
    systemStatusMsg2.buf[0] = aliveCounter;
    systemStatusMsg2.buf[1] = SystemParams.SpeedUnitPref;
    systemStatusMsg2.buf[2] = SystemParams.DistanceUnitPref;
    systemStatusMsg2.buf[3] = SystemParams.AllowData;
    systemStatusMsg2.buf[4] = SystemParams.AllowGPS;
    systemStatusMsg2.buf[5] = SystemParams.AllowMotionDetect;
    systemStatusMsg2.buf[6] = SystemParams.MotionDeadTime;
    Can.write(systemStatusMsg2);
}
