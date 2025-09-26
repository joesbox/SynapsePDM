/*  Display.cpp LCD variables, functions and data handling.
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

#include <Display.h>

#define SCREENWIDTH 320
#define SCREENHEIGHT 240

#define ICON_WIDTH 48
#define ICON_HEIGHT 48

#define CHANNEL_GREY 0xC5C6C5
#include "graphic.h"

SPIClass &spix = SPI;

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

bool backgroundDrawn;
long splashCounter;

static bool prevEnabled[NUM_CHANNELS] = {false};
static int prevErrorFlags[NUM_CHANNELS] = {0};
static float prevCurrentValues[NUM_CHANNELS] = {0.0F};
static bool prevSDOK, prevGPSOK, initIcons = false;
static uint8_t previousConnectionStatus = 0;
static int prevBatt, prevMin = 0;
static uint16_t systemErrorFlags = 0;

const int lights[14][4] = {
    {23, 129, 44, 90},
    {68, 129, 44, 90},
    {113, 129, 44, 90},
    {158, 129, 44, 90},
    {203, 129, 44, 90},
    {248, 129, 44, 90},
    {293, 129, 44, 90},
    {23, 220, 44, 90},
    {68, 220, 44, 90},
    {113, 220, 44, 90},
    {158, 220, 44, 90},
    {203, 220, 44, 90},
    {248, 220, 44, 90},
    {293, 220, 44, 90}};

const int textCoordinates[14][2] = {
    {22, 62},
    {67, 62},
    {112, 62},
    {157, 62},
    {202, 62},
    {247, 62},
    {292, 62},
    {22, 153},
    {67, 153},
    {107, 153},
    {152, 153},
    {197, 153},
    {242, 153},
    {287, 153}};

const int currentReadingCoordinates[14][2] = {
    {10, 82},
    {55, 82},
    {100, 82},
    {145, 82},
    {190, 82},
    {235, 82},
    {280, 82},
    {10, 173},
    {55, 173},
    {100, 173},
    {145, 173},
    {190, 173},
    {235, 173},
    {280, 173}};

const int channelName[14][2] = {
    {22, 102},
    {67, 102},
    {112, 102},
    {157, 102},
    {202, 102},
    {247, 102},
    {292, 102},
    {22, 193},
    {67, 193},
    {113, 193},
    {156, 193},
    {203, 193},
    {248, 193},
    {293, 193}};

void InitialiseDisplay()
{
  tft.begin();
  Serial.print("Enabling DMA: ");
  Serial.println(tft.initDMA());
  spix = tft.getSPIinstance();
  tft.setRotation(3);
  tft.setBitmapColor(TFT_WHITE, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
  tft.pushImage(0, 85, 320, 70, (uint16_t *)epd_bitmap_synapse_logo);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.loadFont(NotoSansBold15);
  tft.setCursor(275, 140);
  tft.print(FW_VER);

  spix.end();

  prevSDOK = SDCardOK;
  prevGPSOK = !GPSFix;
}

void DrawBackground()
{
  spix.begin();

  tft.startWrite();

  tft.fillRect(0, 85, SCREENWIDTH, 100, TFT_BLACK);

  tft.drawLine(0, 58, SCREENWIDTH, 58, TFT_DARKGREY);
  tft.drawLine(0, 148, SCREENWIDTH, 148, TFT_DARKGREY);
  tft.drawLine(0, 238, SCREENWIDTH, 238, TFT_DARKGREY);
  tft.drawLine(0, 58, 0, 238, TFT_DARKGREY);
  tft.drawLine(45, 58, 45, 238, TFT_DARKGREY);
  tft.drawLine(90, 58, 90, 238, TFT_DARKGREY);
  tft.drawLine(135, 58, 135, 238, TFT_DARKGREY);
  tft.drawLine(180, 58, 180, 238, TFT_DARKGREY);
  tft.drawLine(225, 58, 225, 238, TFT_DARKGREY);
  tft.drawLine(270, 58, 270, 238, TFT_DARKGREY);
  tft.drawLine(319, 58, 319, 238, TFT_DARKGREY);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    tft.setCursor(textCoordinates[i][0], textCoordinates[i][1]);
    tft.print(i + 1);
    tft.setCursor(currentReadingCoordinates[i][0], currentReadingCoordinates[i][1]);
    tft.print("0.0A");

    // Calculate the width of the channel name
    int chanNameWidth = tft.textWidth(Channels[i].ChannelName);
    int chanNameX = channelName[i][0] + (5 - chanNameWidth) / 2;

    tft.setCursor(chanNameX, channelName[i][1]);
    tft.print(Channels[i].ChannelName);

    // tft.fillCircle(lights[i][0], lights[i][1], 12, TFT_DARKGREY);
    tft.pushImage(lights[i][0] - 12, lights[i][1] - 12, 24, 24, (uint16_t *)greyLED);
  }

  tft.endWrite();
  spix.end();
  backgroundDrawn = true;
}

void UpdateDisplay()
{
  spix.begin();
  tft.startWrite();

  if (!initIcons)
  {
    initIcons = true;
    // Initial icon states
    tft.pushImage(0, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)logiconError);
    tft.pushImage(49, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)gpsError);
    tft.pushImage(271, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)batteryIcon);
    tft.pushImage(180, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)pc_error);
    tft.setCursor(120, 21);
    tft.fillRect(120, 21, 50, 15, TFT_BLACK);
    tft.print(SystemParams.ErrorFlags, HEX);
    char timeString[6];
    snprintf(timeString, sizeof(timeString), "%02d:%02d", RTChour, RTCminute);
    tft.setCursor(230, 21);
    tft.print(timeString);
  }
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    if (Channels[i].Enabled != prevEnabled[i] || Channels[i].ErrorFlags != prevErrorFlags[i])
    {
      if (Channels[i].Enabled && Channels[i].ErrorFlags == 0)
      {
        tft.pushImage(lights[i][0] - 12, lights[i][1] - 12, 24, 24, (uint16_t *)greenLED);
      }
      else if (Channels[i].Enabled && Channels[i].ErrorFlags != 0)
      {
        tft.pushImage(lights[i][0] - 12, lights[i][1] - 12, 24, 24, (uint16_t *)redLED);
      }
      else
      {
        // tft.fillCircle(lights[i][0], lights[i][1], 12, TFT_DARKGREY);
        tft.pushImage(lights[i][0] - 12, lights[i][1] - 12, 24, 24, (uint16_t *)greyLED);
      }

      // Update previous states
      prevEnabled[i] = Channels[i].Enabled;
      prevErrorFlags[i] = Channels[i].ErrorFlags;
    }

    // Update current values
    float currentValueRounded = round(Channels[i].CurrentValue * 10) / 10.0;
    float prevValueRounded = round(prevCurrentValues[i] * 10) / 10.0;

    if (currentValueRounded != prevValueRounded)
    {
      // Clear the previous text
      tft.fillRect(currentReadingCoordinates[i][0] - 10, currentReadingCoordinates[i][1], 45, 15, TFT_BLACK);

      // Calculate the width of the text
      int textWidth = tft.textWidth(String(currentValueRounded, 1) + "A");

      // Adjust the x-coordinate to center the text
      int xCoordinate = currentReadingCoordinates[i][0] + (30 - textWidth) / 2;

      // Print the new value
      tft.setCursor(xCoordinate, currentReadingCoordinates[i][1]);
      tft.print(currentValueRounded, 1);
      tft.print("A");
      prevCurrentValues[i] = Channels[i].CurrentValue;
    }
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
  }

  // Check for SD card status change
  if (SDCardOK != prevSDOK)
  {
    if (!SDCardOK)
    {
      tft.pushImage(0, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)logiconError);
    }
    else
    {
      tft.pushImage(0, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)logicon);
    }
    prevSDOK = SDCardOK;
  }

  // Check for GPS status change
  if (GPSFix != prevGPSOK)
  {
    if (GPSFix)
    {
      tft.pushImage(49, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)gpsOK);
    }
    else
    {
      tft.pushImage(49, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)gpsError);
    }
    prevGPSOK = GPSFix;
  }

  // Check for PC connection status change
  if (connectionStatus != previousConnectionStatus)
  {
    if (connectionStatus == 1)
    {
      tft.pushImage(180, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)pc_ok);
    }
    else
    {
      tft.pushImage(180, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)pc_error);
    }
    previousConnectionStatus = connectionStatus;
  }

  tft.setCursor(120, 21);
  tft.fillRect(120, 21, 50, 15, TFT_BLACK);
  tft.print(connectionStatus);
  tft.setCursor(120, 35);
  tft.fillRect(120, 35, 50, 15, TFT_BLACK);
  tft.print(recBytesRead);

  /*// Display system error flags
  if (SystemParams.ErrorFlags != systemErrorFlags)
  {
    systemErrorFlags = SystemParams.ErrorFlags;
    tft.setCursor(120, 21);
    tft.fillRect(120, 21, 50, 15, TFT_BLACK);
    tft.print(SystemParams.ErrorFlags, HEX);
  }*/

  if (prevBatt != SOC)
  {
    prevBatt = SOC;
    tft.pushImage(271, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)batteryIcon);
    if (SOC < 100)
    {
      tft.setCursor(282, 22);
    }
    else
    {
      tft.setCursor(278, 22);
    }
    tft.unloadFont();
    tft.loadFont(OpenSans12);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(SOC);
    tft.print("%");
    tft.unloadFont();
    tft.loadFont(NotoSansBold15);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
  }

  if (prevMin != RTCminute)
  {
    char timeString[6];
    snprintf(timeString, sizeof(timeString), "%02d:%02d", RTChour, RTCminute);
    tft.fillRect(229, 21, 40, 15, TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    prevMin = RTCminute;
    tft.setCursor(229, 21);
    tft.print(timeString);
  }
  tft.endWrite();
  spix.end();
}