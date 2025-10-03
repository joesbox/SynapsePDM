/*  Display.h LCD variables, functions and data handling.
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

#ifndef Display_H
#define Display_H

#include <Globals.h>
#include <TFT_eSPI.h>
#include <synapse.h>
#include <NotoSansBold15.h>
#include <OpenSans12.h>

#include <icons.h>

#define USE_DMA_TO_TFT

/// @brief Flag to denote if the background has been drawn
extern bool backgroundDrawn;

/// @brief Counter for splash screen delay
extern long splashCounter;

/// @brief Initialise LCD
void InitialiseDisplay();

/// @brief Draw the initial background
void DrawBackground();

/// @brief Update the display
void UpdateDisplay();

#endif