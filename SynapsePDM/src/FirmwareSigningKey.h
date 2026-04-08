/*  FirmwareSigningKey.h Firmware signing key definitions and functions.
    Copyright (c) 2026 Joe Mann.  All right reserved.

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

#ifndef FirmwareSigningKey_H
#define FirmwareSigningKey_H

#include <Arduino.h>

#define FIRMWARE_UPDATE_PUBLIC_KEY_SIZE 32

static const uint8_t FIRMWARE_UPDATE_PUBLIC_KEY[FIRMWARE_UPDATE_PUBLIC_KEY_SIZE] = {
    0xde, 0x90, 0xd1, 0xb1, 0xb1, 0x3a, 0x6b, 0x63,
    0xa4, 0x28, 0x72, 0xb2, 0x14, 0x83, 0x94, 0x93,
    0x5c, 0x5f, 0x3d, 0x40, 0x56, 0x52, 0xf8, 0xa5,
    0x0f, 0xf1, 0x49, 0x07, 0xad, 0x2f, 0x44, 0x2b,
};

#endif