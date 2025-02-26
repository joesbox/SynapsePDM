/*  InputHandler.cpp Input handler deals with digital channel input status.
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

#include "InputHandler.h"

void InitialiseInputs()
{
    // Ignition inout is used for wake/sleep
    pinMode(IGN_INPUT, INPUT_PULLDOWN);

    // Digital inputs. Default to active-high
    for (int i = 0; i < NUM_DI_CHANNELS; i++)
    {
        pinMode(DIchannelInputPins[i], INPUT_PULLDOWN);
    }

    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        AnalogueIns[i].InputPin = ANAchannelInputPins[i];
        AnalogueIns[i].PullDownPin = ANAchannelInputPullDowns[i];
        AnalogueIns[i].PullUpPin = ANAchannelInputPullUps[i];

        pinMode(AnalogueIns[i].PullDownPin, OUTPUT);
        pinMode(AnalogueIns[i].PullUpPin, OUTPUT);
    }
}

void HandleInputs()
{
    // Check channel type and enable for active level
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        switch (Channels[i].ChanType)
        {
        case DIG_PWM:
        case DIG:
            // Channels[i].Enabled = digitalRead(Channels[i].InputControlPin);
            break;
        default:
            break;
        }
    }

    // Check analogue inputs. Set pull-ups/pull-downs
    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        digitalWrite(AnalogueIns[i].PullDownPin, AnalogueIns[i].PullDownEnable);
        digitalWrite(AnalogueIns[i].PullUpPin, AnalogueIns[i].PullUpEnable);
    }
}

void PullResistorSleep()
{
    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        digitalWrite(AnalogueIns[i].PullDownPin, LOW);
        digitalWrite(AnalogueIns[i].PullUpPin, LOW);
    }
}