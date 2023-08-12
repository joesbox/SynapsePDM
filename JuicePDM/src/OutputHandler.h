/*  OutputHandler.h Output handler deals with channel output control.
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

#ifndef OutputHandler_H
#define OutputHandler_H

#include <Arduino.h>
#include <ChannelConfig.h>
#include <SoftPWM.h>

/// @brief Output handler class
class OutputHandler
{
public:
    OutputHandler(int numChannels); // Constructor
    
    /// @brief Channel configurations
    ChannelConfig* Channels;

    /// @brief Initialize channels
    void Initialize();

    /// @brief Run channels at their set PWM or output state
    void SetOutputs();

private:
    /// @brief Number of channels
    int numberChannels;
};

#endif
