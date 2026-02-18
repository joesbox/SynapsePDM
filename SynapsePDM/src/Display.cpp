/*  Display.cpp LCD variables, functions and data handling.
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

#include <Display.h>

#define SCREENWIDTH 320
#define SCREENHEIGHT 240

#define ICON_WIDTH 48
#define ICON_HEIGHT 48

#define CHANNEL_GREY 0xC5C6C5

SPIClass &spix = SPI;

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

long splashCounter;

static bool prevEnabled[NUM_CHANNELS] = {false};
static int prevErrorFlags[NUM_CHANNELS] = {0};
static float prevCurrentValues[NUM_CHANNELS] = {0.0F};
static bool prevSDOK = false, prevGPSOK = false, initIcons = false, prevGPSEnable = false, previousConnectionStatus = false;
static uint8_t prevMotionStatus = 0;
static int prevMin = 0;
static uint16_t systemErrorFlags = 0;
static uint8_t prevBars = 0;

bool invalidateDisplay = false;

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
  digitalWrite(TFT_RST, LOW);
  delay(20);
  digitalWrite(TFT_RST, HIGH);
  tft.begin();
  tft.initDMA();
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

void StartDisplay()
{
  initIcons = false;
  SPI_2.begin();
  digitalWrite(TFT_RST, LOW);
  delay(20);
  digitalWrite(TFT_RST, HIGH);
  tft.begin();
  tft.initDMA();
  spix = tft.getSPIinstance();
  tft.setRotation(3);
  tft.setBitmapColor(TFT_WHITE, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
  spix.end();
}

void StopDisplay()
{
  // End SPI communication
  tft.dmaWait();
  tft.endWrite();

  if (DMA1_Stream4->CR & DMA_SxCR_EN)
  {
    DMA1_Stream4->CR &= ~DMA_SxCR_EN; // disable DMA stream
    while (DMA1_Stream4->CR & DMA_SxCR_EN)
    {
    } // wait for it to actually disable
  }

  SPI_2.end();
  spix.end();

  pinMode(PICO, OUTPUT);
  pinMode(POCI, OUTPUT);
  pinMode(SCK2, OUTPUT);
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_RST, OUTPUT);
  pinMode(PB10, OUTPUT);
  digitalWrite(CS1, LOW);
  digitalWrite(CS2, LOW);
  digitalWrite(PICO, LOW);
  digitalWrite(POCI, LOW);
  digitalWrite(SCK2, LOW);
  digitalWrite(TFT_DC, LOW);
  digitalWrite(TFT_RST, LOW);
  digitalWrite(PWR_EN_5V, LOW);
  digitalWrite(PWR_EN_3V3, LOW);
  digitalWrite(PB10, LOW);
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

    // Ensure channel name is null-terminated
    char safeName[4];
    memcpy(safeName, Channels[i].ChannelName, 3);
    safeName[3] = '\0';

    tft.setCursor(chanNameX, channelName[i][1]);
    tft.print(safeName);

    tft.pushImage(lights[i][0] - 10, lights[i][1] - 8, 24, 24, (uint16_t *)greyLED);
  }

  tft.endWrite();
  spix.end();
  backgroundDrawn = true;
}

void UpdateDisplay()
{
  spix.begin();
  tft.startWrite();

  if (invalidateDisplay)
  {
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
      prevEnabled[i] = !Channels[i].Enabled;
      prevErrorFlags[i] = -1;
      prevCurrentValues[i] = -1.0F;

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

      // Calculate the width of the channel name
      int chanNameWidth = tft.textWidth(Channels[i].ChannelName);
      int chanNameX = channelName[i][0] + (5 - chanNameWidth) / 2;

      // Ensure channel name is null-terminated
      char safeName[4];
      memcpy(safeName, Channels[i].ChannelName, 3);
      safeName[3] = '\0';

      tft.fillRect(chanNameX, channelName[i][1], chanNameWidth, tft.fontHeight(), TFT_BLACK);
      tft.setCursor(chanNameX, channelName[i][1]);
      tft.print(safeName);
    }

    prevSDOK = !SDCardOK;
    prevGPSOK = !GPSFix;
    prevMotionStatus = !SystemParams.AllowMotionDetect;
    systemErrorFlags = !SystemRuntimeParams.ErrorFlags;
    previousConnectionStatus = !pcCommsOK;
    invalidateDisplay = false;
  }

  if (!initIcons)
  {
    initIcons = true;
    // Initial icon states
    tft.pushImage(0, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)logiconError);
    if (SystemParams.AllowGPS)
    {
      if (GPSFix)
      {
        tft.pushImage(47, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)gpsOK);
      }
      else
      {
        tft.pushImage(47, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)gpsError);
      }
    }
    else
    {
      tft.fillRect(47, 4, ICON_WIDTH, ICON_HEIGHT, TFT_BLACK);
    }

    if (SystemParams.AllowMotionDetect != 0)
    {
      tft.pushImage(99, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)motion_ok);
    }
    else
    {
      tft.pushImage(99, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)motion_error);
    }

    tft.pushImage(151, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)zero_bar);
    char timeString[6];
    snprintf(timeString, sizeof(timeString), "%02d:%02d", rtc.getHours(), rtc.getMinutes());
    tft.setCursor(271, 21);
    tft.print(timeString);
  }
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    if (Channels[i].Enabled != prevEnabled[i] || ChannelRuntime[i].ErrorFlags != prevErrorFlags[i])
    {
      if (Channels[i].Enabled && ChannelRuntime[i].ErrorFlags == 0)
      {
        tft.pushImage(lights[i][0] - 10, lights[i][1] - 8, 24, 24, (uint16_t *)greenLED);
      }
      else if (Channels[i].Enabled && ChannelRuntime[i].ErrorFlags != 0)
      {
        tft.pushImage(lights[i][0] - 10, lights[i][1] - 8, 24, 24, (uint16_t *)redLED);
      }
      else
      {
        tft.pushImage(lights[i][0] - 10, lights[i][1] - 8, 24, 24, (uint16_t *)greyLED);
      }

      // Update previous states
      prevEnabled[i] = Channels[i].Enabled;
      prevErrorFlags[i] = ChannelRuntime[i].ErrorFlags;
    }

    // Update current values
    float currentValueRounded = round(ChannelRuntime[i].CurrentValue * 10) / 10.0;
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
      prevCurrentValues[i] = ChannelRuntime[i].CurrentValue;
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
  if (GPSFix != prevGPSOK || SystemParams.AllowGPS != prevGPSEnable)
  {
    if (SystemParams.AllowGPS)
    {
      if (GPSFix)
      {
        tft.pushImage(47, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)gpsOK);
      }
      else
      {
        tft.pushImage(47, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)gpsError);
      }
    }
    else
    {
      tft.fillRect(47, 4, ICON_WIDTH, ICON_HEIGHT, TFT_BLACK);
    }
    prevGPSOK = GPSFix;
    prevGPSEnable = SystemParams.AllowGPS;
  }

  // Check for motion detection status change
  if (SystemParams.AllowMotionDetect != prevMotionStatus)
  {
    if (SystemParams.AllowMotionDetect != 0)
    {
      tft.pushImage(99, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)motion_ok);
    }
    else
    {
      tft.pushImage(99, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)motion_error);
    }
    prevMotionStatus = SystemParams.AllowMotionDetect;
  }

  // Check for system error flags change
  if (SystemRuntimeParams.ErrorFlags != systemErrorFlags)
  {
    int16_t textWidth = tft.textWidth("EFFFF");
    int16_t textHeight = tft.fontHeight();
    if (SystemRuntimeParams.ErrorFlags != 0)
    {
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.fillRect(269, 40, textWidth + 5, textHeight, TFT_BLACK);
      tft.setCursor(269, 40);
      tft.printf("E%04X", SystemRuntimeParams.ErrorFlags);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
    }
    else
    {
      tft.fillRect(269, 40, textWidth + 5, textHeight, TFT_BLACK);
    }
    systemErrorFlags = SystemRuntimeParams.ErrorFlags;
  }

  // Check for PC connection status change
  if (pcCommsOK != previousConnectionStatus)
  {
    if (pcCommsOK)
    {
      tft.pushImage(220, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)pc_ok);
    }
    else
    {
      tft.fillRect(220, 4, ICON_WIDTH, ICON_HEIGHT, TFT_BLACK);
    }
    previousConnectionStatus = pcCommsOK;
  }

  // Check for signal bars change
  int bars = csq_to_bars();
  if (bars != prevBars)
  {
    switch (bars)
    {
    case 0:
      tft.pushImage(151, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)zero_bar);
      break;
    case 1:
      tft.pushImage(151, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)one_bar);
      break;
    case 2:
      tft.pushImage(151, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)two_bar);
      break;
    case 3:
      tft.pushImage(151, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)three_bar);
      break;
    case 4:
      tft.pushImage(151, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)four_bar);
      break;
    case 5:
      tft.pushImage(151, 4, ICON_WIDTH, ICON_HEIGHT, (uint16_t *)five_bar);
      break;
    }
    prevBars = bars;
  }

  if (prevMin != rtc.getMinutes())
  {
    char timeString[6];
    snprintf(timeString, sizeof(timeString), "%02d:%02d", rtc.getHours(), rtc.getMinutes());
    tft.fillRect(271, 21, 40, 15, TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    prevMin = rtc.getMinutes();
    tft.setCursor(271, 21);
    tft.print(timeString);
  }
  tft.endWrite();
  spix.end();
}