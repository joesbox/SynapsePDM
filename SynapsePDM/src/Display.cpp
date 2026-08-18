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
#include <OutputHandler.h>

#define SCREENWIDTH 320
#define SCREENHEIGHT 240
#define ICON_WIDTH 48
#define ICON_HEIGHT 48
#define DISPLAY_SPI_FILL_BUFFER_SIZE 2048U
#define DISPLAY_SPI_FILL_BUFFER_PIXELS (DISPLAY_SPI_FILL_BUFFER_SIZE / 2U)
#define DISPLAY_STATIC_LABEL_MAX_HEIGHT 20
#define DISPLAY_STATIC_NUMBER_MAX_WIDTH 16
#define DISPLAY_STATIC_NAME_MAX_WIDTH 32

#define ILI9341_CASET 0x2A
#define ILI9341_PASET 0x2B
#define ILI9341_RAMWR 0x2C

#ifndef DISPLAY_WAKE_CLEAR_COLOR
#define DISPLAY_WAKE_CLEAR_COLOR TFT_BLACK
#endif

#define CHANNEL_GREY 0xC5C6C5

SPIClass &spix = SPI;

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

static uint16_t displaySpiFillBuffer[DISPLAY_SPI_FILL_BUFFER_PIXELS];
static TFT_eSprite initialCurrentLabelSprite = TFT_eSprite(&tft);
static TFT_eSprite textLabelStageSprite = TFT_eSprite(&tft);
static uint16_t *initialCurrentLabelPixels = nullptr;
static int initialCurrentLabelWidth = 0;
static int initialCurrentLabelHeight = 0;
static uint16_t channelNumberLabelPixels[NUM_CHANNELS][DISPLAY_STATIC_NUMBER_MAX_WIDTH * DISPLAY_STATIC_LABEL_MAX_HEIGHT];
static uint16_t channelNameLabelPixels[NUM_CHANNELS][DISPLAY_STATIC_NAME_MAX_WIDTH * DISPLAY_STATIC_LABEL_MAX_HEIGHT];
static uint8_t channelNumberLabelWidths[NUM_CHANNELS] = {0};
static uint8_t channelNumberLabelHeights[NUM_CHANNELS] = {0};
static uint8_t channelNameLabelWidths[NUM_CHANNELS] = {0};
static uint8_t channelNameLabelHeights[NUM_CHANNELS] = {0};
static bool channelNumberLabelValid[NUM_CHANNELS] = {false};
static bool channelNameLabelValid[NUM_CHANNELS] = {false};
static char cachedChannelNames[NUM_CHANNELS][4] = {{0}};

long splashCounter;

static bool prevEnabled[NUM_CHANNELS] = {false};
static int prevErrorFlags[NUM_CHANNELS] = {0};
static float prevCurrentValues[NUM_CHANNELS] = {0.0F};
static bool prevSDOK = false, prevGPSOK = false, initIcons = false, prevGPSEnable = false, previousConnectionStatus = false;
static uint8_t prevMotionStatus = 0;
static int prevHour = -1;
static int prevMin = -1;
static uint16_t systemErrorFlags = 0;
static uint8_t prevBars = 0;
static bool backgroundClearNeeded = true;

static void DisplayWriteCommandBuffer(SPI_HandleTypeDef *spiHandle, uint8_t command, const uint8_t *data, uint16_t size)
{
  digitalWrite(TFT_CS, LOW);
  digitalWrite(TFT_DC, LOW);
  HAL_SPI_Transmit(spiHandle, &command, 1U, HAL_MAX_DELAY);

  if ((data != nullptr) && (size > 0U))
  {
    digitalWrite(TFT_DC, HIGH);
    HAL_SPI_Transmit(spiHandle, (uint8_t *)data, size, HAL_MAX_DELAY);
  }

  digitalWrite(TFT_CS, HIGH);
}

static void DisplaySetAddressWindowRaw(SPI_HandleTypeDef *spiHandle, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  uint8_t data[4];

  data[0] = (uint8_t)(x0 >> 8);
  data[1] = (uint8_t)(x0 & 0xFFU);
  data[2] = (uint8_t)(x1 >> 8);
  data[3] = (uint8_t)(x1 & 0xFFU);
  DisplayWriteCommandBuffer(spiHandle, ILI9341_CASET, data, sizeof(data));

  data[0] = (uint8_t)(y0 >> 8);
  data[1] = (uint8_t)(y0 & 0xFFU);
  data[2] = (uint8_t)(y1 >> 8);
  data[3] = (uint8_t)(y1 & 0xFFU);
  DisplayWriteCommandBuffer(spiHandle, ILI9341_PASET, data, sizeof(data));
}

static bool PushImageRaw16Bit(int32_t x, int32_t y, int32_t width, int32_t height, const uint16_t *image)
{
  if ((image == nullptr) || (width <= 0) || (height <= 0))
  {
    return false;
  }

  spix.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, TFT_SPI_MODE));

  SPI_HandleTypeDef *spiHandle = spix.getHandle();
  uint32_t remainingPixels = (uint32_t)width * (uint32_t)height;
  uint32_t originalDataSize = SPI_DATASIZE_8BIT;
  bool use16BitData = false;

  if (spiHandle == nullptr)
  {
    spix.endTransaction();
    return false;
  }

  originalDataSize = spiHandle->Init.DataSize;

  DisplaySetAddressWindowRaw(spiHandle, (uint16_t)x, (uint16_t)y, (uint16_t)(x + width - 1), (uint16_t)(y + height - 1));

  digitalWrite(TFT_CS, LOW);
  digitalWrite(TFT_DC, LOW);
  uint8_t command = ILI9341_RAMWR;
  HAL_SPI_Transmit(spiHandle, &command, 1U, HAL_MAX_DELAY);
  digitalWrite(TFT_DC, HIGH);

  if (spiHandle->Init.DataSize != SPI_DATASIZE_16BIT)
  {
    __HAL_SPI_DISABLE(spiHandle);
    spiHandle->Init.DataSize = SPI_DATASIZE_16BIT;
    if (HAL_SPI_Init(spiHandle) == HAL_OK)
    {
      use16BitData = true;
    }
    else
    {
      spiHandle->Init.DataSize = originalDataSize;
      (void)HAL_SPI_Init(spiHandle);
    }
  }

  while (remainingPixels > 0U)
  {
    uint32_t chunkPixels = remainingPixels;
    if (chunkPixels > DISPLAY_SPI_FILL_BUFFER_PIXELS)
    {
      chunkPixels = DISPLAY_SPI_FILL_BUFFER_PIXELS;
    }

    for (uint32_t i = 0; i < chunkPixels; i++)
    {
      uint16_t pixel = image[i];
      displaySpiFillBuffer[i] = (uint16_t)((pixel >> 8) | (pixel << 8));
    }

    if (use16BitData)
    {
      HAL_SPI_Transmit(spiHandle, (uint8_t *)displaySpiFillBuffer, (uint16_t)chunkPixels, HAL_MAX_DELAY);
    }
    else
    {
      HAL_SPI_Transmit(spiHandle, (uint8_t *)displaySpiFillBuffer, (uint16_t)(chunkPixels * 2U), HAL_MAX_DELAY);
    }

    image += chunkPixels;
    remainingPixels -= chunkPixels;
  }

  if (spiHandle->Init.DataSize != originalDataSize)
  {
    __HAL_SPI_DISABLE(spiHandle);
    spiHandle->Init.DataSize = originalDataSize;
    (void)HAL_SPI_Init(spiHandle);
  }

  digitalWrite(TFT_CS, HIGH);
  spix.endTransaction();
  return true;
}

static void PushImageRaw16BitWithFallback(int32_t x, int32_t y, int32_t width, int32_t height, const uint16_t *image)
{
  if (!PushImageRaw16Bit(x, y, width, height, image))
  {
    spix.begin();
    tft.startWrite();
    tft.pushImage(x, y, width, height, (uint16_t *)image);
    tft.endWrite();
    spix.end();
  }
}

static void PushImageRaw16BitInterruptingWrite(int32_t x, int32_t y, int32_t width, int32_t height, const uint16_t *image)
{
  tft.endWrite();
  spix.end();
  PushImageRaw16BitWithFallback(x, y, width, height, image);
  spix.begin();
  tft.startWrite();
}

static bool EnsureInitialCurrentLabelSprite()
{
  static const char initialCurrentText[] = "0A";

  if ((initialCurrentLabelPixels != nullptr) && (initialCurrentLabelWidth > 0) && (initialCurrentLabelHeight > 0))
  {
    return true;
  }

  initialCurrentLabelWidth = tft.textWidth(initialCurrentText);
  initialCurrentLabelHeight = tft.fontHeight();

  if ((initialCurrentLabelWidth <= 0) || (initialCurrentLabelHeight <= 0))
  {
    initialCurrentLabelPixels = nullptr;
    return false;
  }

  initialCurrentLabelSprite.setColorDepth(16);
  if (initialCurrentLabelSprite.createSprite(initialCurrentLabelWidth, initialCurrentLabelHeight) == nullptr)
  {
    initialCurrentLabelPixels = nullptr;
    return false;
  }

  initialCurrentLabelSprite.loadFont(NotoSansBold15);
  initialCurrentLabelSprite.setTextColor(TFT_WHITE, TFT_BLACK);
  initialCurrentLabelSprite.fillSprite(TFT_BLACK);
  initialCurrentLabelSprite.drawString(initialCurrentText, 0, 0);

  initialCurrentLabelPixels = (uint16_t *)initialCurrentLabelSprite.getPointer();
  if (initialCurrentLabelPixels == nullptr)
  {
    initialCurrentLabelSprite.deleteSprite();
    initialCurrentLabelWidth = 0;
    initialCurrentLabelHeight = 0;
    return false;
  }

  return true;
}

static bool RenderTextLabelToBuffer(const char *text, uint16_t *labelPixels, int maxWidth, int maxHeight, int *labelWidth, int *labelHeight)
{
  if ((text == nullptr) || (labelPixels == nullptr) || (labelWidth == nullptr) || (labelHeight == nullptr))
  {
    return false;
  }

  int renderedWidth = tft.textWidth(text);
  int renderedHeight = tft.fontHeight();

  if ((renderedWidth <= 0) || (renderedHeight <= 0) || (renderedWidth > maxWidth) || (renderedHeight > maxHeight))
  {
    *labelWidth = 0;
    *labelHeight = 0;
    return false;
  }

  if (textLabelStageSprite.created())
  {
    textLabelStageSprite.deleteSprite();
  }

  textLabelStageSprite.setColorDepth(16);
  if (textLabelStageSprite.createSprite(renderedWidth, renderedHeight) == nullptr)
  {
    *labelWidth = 0;
    *labelHeight = 0;
    return false;
  }

  textLabelStageSprite.loadFont(NotoSansBold15);
  textLabelStageSprite.setTextColor(TFT_WHITE, TFT_BLACK);
  textLabelStageSprite.fillSprite(TFT_BLACK);
  textLabelStageSprite.drawString(text, 0, 0);

  uint16_t *sourcePixels = (uint16_t *)textLabelStageSprite.getPointer();
  if (sourcePixels == nullptr)
  {
    textLabelStageSprite.deleteSprite();
    *labelWidth = 0;
    *labelHeight = 0;
    return false;
  }

  memcpy(labelPixels, sourcePixels, (size_t)renderedWidth * (size_t)renderedHeight * sizeof(uint16_t));
  textLabelStageSprite.deleteSprite();

  *labelWidth = renderedWidth;
  *labelHeight = renderedHeight;
  return true;
}

static bool EnsureChannelNumberLabel(int channelIndex)
{
  if ((channelIndex < 0) || (channelIndex >= NUM_CHANNELS))
  {
    return false;
  }

  if (channelNumberLabelValid[channelIndex])
  {
    return true;
  }

  char numberText[3];
  if (channelIndex < 9)
  {
    numberText[0] = (char)('1' + channelIndex);
    numberText[1] = '\0';
  }
  else
  {
    numberText[0] = '1';
    numberText[1] = (char)('0' + (channelIndex - 9));
    numberText[2] = '\0';
  }

  int labelWidth = 0;
  int labelHeight = 0;
  if (!RenderTextLabelToBuffer(numberText, channelNumberLabelPixels[channelIndex], DISPLAY_STATIC_NUMBER_MAX_WIDTH, DISPLAY_STATIC_LABEL_MAX_HEIGHT, &labelWidth, &labelHeight))
  {
    channelNumberLabelValid[channelIndex] = false;
    channelNumberLabelWidths[channelIndex] = 0;
    channelNumberLabelHeights[channelIndex] = 0;
    return false;
  }

  channelNumberLabelWidths[channelIndex] = (uint8_t)labelWidth;
  channelNumberLabelHeights[channelIndex] = (uint8_t)labelHeight;
  channelNumberLabelValid[channelIndex] = true;
  return true;
}

static bool EnsureChannelNameLabel(int channelIndex)
{
  if ((channelIndex < 0) || (channelIndex >= NUM_CHANNELS))
  {
    return false;
  }

  char safeName[4];
  memcpy(safeName, Channels[channelIndex].ChannelName, 3);
  safeName[3] = '\0';

  if (channelNameLabelValid[channelIndex] && (memcmp(cachedChannelNames[channelIndex], safeName, sizeof(safeName)) == 0))
  {
    return true;
  }

  int labelWidth = 0;
  int labelHeight = 0;
  if (!RenderTextLabelToBuffer(safeName, channelNameLabelPixels[channelIndex], DISPLAY_STATIC_NAME_MAX_WIDTH, DISPLAY_STATIC_LABEL_MAX_HEIGHT, &labelWidth, &labelHeight))
  {
    channelNameLabelValid[channelIndex] = false;
    channelNameLabelWidths[channelIndex] = 0;
    channelNameLabelHeights[channelIndex] = 0;
    return false;
  }

  memcpy(cachedChannelNames[channelIndex], safeName, sizeof(safeName));
  channelNameLabelWidths[channelIndex] = (uint8_t)labelWidth;
  channelNameLabelHeights[channelIndex] = (uint8_t)labelHeight;
  channelNameLabelValid[channelIndex] = true;
  return true;
}

static bool FillScreenWakeRaw(uint16_t color)
{
  spix.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, TFT_SPI_MODE));

  SPI_HandleTypeDef *spiHandle = spix.getHandle();
  uint32_t remainingPixels = (uint32_t)SCREENWIDTH * SCREENHEIGHT;
  uint32_t originalDataSize = SPI_DATASIZE_8BIT;
  bool use16BitData = false;

  if (spiHandle == nullptr)
  {
    spix.endTransaction();
    return false;
  }

  originalDataSize = spiHandle->Init.DataSize;

  for (uint32_t index = 0; index < DISPLAY_SPI_FILL_BUFFER_PIXELS; index++)
  {
    displaySpiFillBuffer[index] = color;
  }

  DisplaySetAddressWindowRaw(spiHandle, 0U, 0U, (uint16_t)(SCREENWIDTH - 1), (uint16_t)(SCREENHEIGHT - 1));

  digitalWrite(TFT_CS, LOW);
  digitalWrite(TFT_DC, LOW);
  uint8_t command = ILI9341_RAMWR;
  HAL_SPI_Transmit(spiHandle, &command, 1U, HAL_MAX_DELAY);
  digitalWrite(TFT_DC, HIGH);

  if (spiHandle->Init.DataSize != SPI_DATASIZE_16BIT)
  {
    __HAL_SPI_DISABLE(spiHandle);
    spiHandle->Init.DataSize = SPI_DATASIZE_16BIT;
    if (HAL_SPI_Init(spiHandle) == HAL_OK)
    {
      use16BitData = true;
    }
    else
    {
      spiHandle->Init.DataSize = originalDataSize;
      (void)HAL_SPI_Init(spiHandle);
    }
  }

  while (remainingPixels > 0U)
  {
    uint32_t chunkPixels = remainingPixels;
    if (chunkPixels > DISPLAY_SPI_FILL_BUFFER_PIXELS)
    {
      chunkPixels = DISPLAY_SPI_FILL_BUFFER_PIXELS;
    }

    if (use16BitData)
    {
      HAL_SPI_Transmit(spiHandle, (uint8_t *)displaySpiFillBuffer, (uint16_t)chunkPixels, HAL_MAX_DELAY);
    }
    else
    {
      HAL_SPI_Transmit(spiHandle, (uint8_t *)displaySpiFillBuffer, (uint16_t)(chunkPixels * 2U), HAL_MAX_DELAY);
    }
    remainingPixels -= chunkPixels;
  }

  if (spiHandle->Init.DataSize != originalDataSize)
  {
    __HAL_SPI_DISABLE(spiHandle);
    spiHandle->Init.DataSize = originalDataSize;
    (void)HAL_SPI_Init(spiHandle);
  }

  digitalWrite(TFT_CS, HIGH);
  spix.endTransaction();
  return true;
}

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
  delay(5);
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
  backgroundClearNeeded = true;
}

void StartDisplay()
{
  initIcons = false;
  SPI_2.begin();
  tft.begin();
  tft.initDMA();
  spix = tft.getSPIinstance();
  tft.setRotation(3);
  tft.setBitmapColor(TFT_WHITE, TFT_BLACK);
  if (!FillScreenWakeRaw(DISPLAY_WAKE_CLEAR_COLOR))
  {
    tft.fillScreen(DISPLAY_WAKE_CLEAR_COLOR);
  }
  backgroundClearNeeded = false;
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
  digitalWrite(PB10, LOW);
}

void DrawBackground()
{
  static const char initialCurrentText[] = "0A";
  bool haveInitialCurrentLabelSprite = EnsureInitialCurrentLabelSprite();
  bool haveChannelNumberLabels[NUM_CHANNELS];
  bool haveChannelNameLabels[NUM_CHANNELS];

  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    haveChannelNumberLabels[i] = EnsureChannelNumberLabel(i);
    haveChannelNameLabels[i] = EnsureChannelNameLabel(i);
  }

  spix.begin();

  tft.startWrite();

  if (backgroundClearNeeded)
  {
    tft.fillRect(0, 58, SCREENWIDTH, 181, TFT_BLACK);
  }

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

  int initialTextWidth = initialCurrentLabelWidth;
  int initialTextHeight = initialCurrentLabelHeight;

  if (!haveInitialCurrentLabelSprite)
  {
    initialTextWidth = tft.textWidth(initialCurrentText);
    initialTextHeight = tft.fontHeight();
  }

  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    (void)i;
  }

  tft.endWrite();
  spix.end();

  if (haveInitialCurrentLabelSprite)
  {
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
      int ledCenterX = lights[i][0] + 2;
      int currentTextX = ledCenterX - (initialTextWidth / 2);
      int currentTextY = currentReadingCoordinates[i][1];

      if (!PushImageRaw16Bit(currentTextX, currentTextY, initialTextWidth, initialTextHeight, initialCurrentLabelPixels))
      {
        spix.begin();
        tft.startWrite();
        tft.pushImage(currentTextX, currentTextY, initialTextWidth, initialTextHeight, initialCurrentLabelPixels);
        tft.endWrite();
        spix.end();
      }
    }
  }
  else
  {
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
      int ledCenterX = lights[i][0] + 2;
      int currentTextY = currentReadingCoordinates[i][1];
      spix.begin();
      tft.startWrite();
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setCursor(ledCenterX - (initialTextWidth / 2), currentTextY);
      tft.print(initialCurrentText);
      tft.endWrite();
      spix.end();
    }
  }

  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    if (haveChannelNumberLabels[i])
    {
      if (!PushImageRaw16Bit(textCoordinates[i][0], textCoordinates[i][1], channelNumberLabelWidths[i], channelNumberLabelHeights[i], channelNumberLabelPixels[i]))
      {
        spix.begin();
        tft.startWrite();
        tft.pushImage(textCoordinates[i][0], textCoordinates[i][1], channelNumberLabelWidths[i], channelNumberLabelHeights[i], channelNumberLabelPixels[i]);
        tft.endWrite();
        spix.end();
      }
    }
    else
    {
      spix.begin();
      tft.startWrite();
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setCursor(textCoordinates[i][0], textCoordinates[i][1]);
      tft.print(i + 1);
      tft.endWrite();
      spix.end();
    }

    if (haveChannelNameLabels[i])
    {
      int chanNameX = channelName[i][0] + (5 - channelNameLabelWidths[i]) / 2;
      if (!PushImageRaw16Bit(chanNameX, channelName[i][1], channelNameLabelWidths[i], channelNameLabelHeights[i], channelNameLabelPixels[i]))
      {
        spix.begin();
        tft.startWrite();
        tft.pushImage(chanNameX, channelName[i][1], channelNameLabelWidths[i], channelNameLabelHeights[i], channelNameLabelPixels[i]);
        tft.endWrite();
        spix.end();
      }
    }
    else
    {
      char safeName[4];
      memcpy(safeName, Channels[i].ChannelName, 3);
      safeName[3] = '\0';

      int chanNameWidth = tft.textWidth(safeName);
      int chanNameX = channelName[i][0] + (5 - chanNameWidth) / 2;

      spix.begin();
      tft.startWrite();
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setCursor(chanNameX, channelName[i][1]);
      tft.print(safeName);
      tft.endWrite();
      spix.end();
    }
  }

  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    if (!PushImageRaw16Bit(lights[i][0] - 10, lights[i][1] - 8, 24, 24, greyLED))
    {
      spix.begin();
      tft.startWrite();
      tft.pushImage(lights[i][0] - 10, lights[i][1] - 8, 24, 24, (uint16_t *)greyLED);
      tft.endWrite();
      spix.end();
    }
  }

  backgroundClearNeeded = true;
  backgroundDrawn = true;
}

void UpdateDisplay()
{
  spix.begin();
  tft.startWrite();
  bool outputsInhibited = AreOutputsInhibited();

  if (invalidateDisplay)
  {
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
      bool effectiveEnabled = IsChannelEffectivelyEnabled(i);
      prevEnabled[i] = !effectiveEnabled;
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

      char safeName[4];
      memcpy(safeName, Channels[i].ChannelName, 3);
      safeName[3] = '\0';

      int chanNameWidth = tft.textWidth(safeName);
      int chanNameX = channelName[i][0] + (5 - chanNameWidth) / 2;

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
    PushImageRaw16BitInterruptingWrite(0, 4, ICON_WIDTH, ICON_HEIGHT, logiconError);
    if (SystemParams.AllowGPS)
    {
      if (GPSFix)
      {
        PushImageRaw16BitInterruptingWrite(47, 4, ICON_WIDTH, ICON_HEIGHT, gpsOK);
      }
      else
      {
        PushImageRaw16BitInterruptingWrite(47, 4, ICON_WIDTH, ICON_HEIGHT, gpsError);
      }
    }
    else
    {
      tft.fillRect(47, 4, ICON_WIDTH, ICON_HEIGHT, TFT_BLACK);
    }

    if (SystemParams.AllowMotionDetect != 0)
    {
      PushImageRaw16BitInterruptingWrite(99, 4, ICON_WIDTH, ICON_HEIGHT, motion_ok);
    }
    else
    {
      PushImageRaw16BitInterruptingWrite(99, 4, ICON_WIDTH, ICON_HEIGHT, motion_error);
    }

    PushImageRaw16BitInterruptingWrite(151, 4, ICON_WIDTH, ICON_HEIGHT, zero_bar);
    char timeString[6];
    snprintf(timeString, sizeof(timeString), "%02d:%02d", rtc.getHours(), rtc.getMinutes());
    tft.setCursor(271, 21);
    tft.print(timeString);
    prevHour = rtc.getHours();
    prevMin = rtc.getMinutes();
  }
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    bool effectiveEnabled = IsChannelEffectivelyEnabled(i);
    int effectiveErrorFlags = outputsInhibited ? 0 : ChannelRuntime[i].ErrorFlags;

    if (effectiveEnabled != prevEnabled[i] || effectiveErrorFlags != prevErrorFlags[i])
    {
      if (effectiveEnabled && effectiveErrorFlags == 0)
      {
        tft.pushImage(lights[i][0] - 10, lights[i][1] - 8, 24, 24, (uint16_t *)greenLED);
      }
      else if (effectiveEnabled && effectiveErrorFlags != 0)
      {
        tft.pushImage(lights[i][0] - 10, lights[i][1] - 8, 24, 24, (uint16_t *)redLED);
      }
      else
      {
        tft.pushImage(lights[i][0] - 10, lights[i][1] - 8, 24, 24, (uint16_t *)greyLED);
      }

      // Update previous states
      prevEnabled[i] = effectiveEnabled;
      prevErrorFlags[i] = effectiveErrorFlags;
    }

    // Update current values
    int currentValueRoundedUp = (ChannelRuntime[i].CurrentValue <= 0.0F) ? 0 : (int)ceil(ChannelRuntime[i].CurrentValue);
    int prevValueRoundedUp = (prevCurrentValues[i] <= 0.0F) ? 0 : (int)ceil(prevCurrentValues[i]);

    if (currentValueRoundedUp != prevValueRoundedUp)
    {
      int ledCenterX = lights[i][0] + 2;
      int currentTextY = currentReadingCoordinates[i][1];

      // Keep clear area inside the cell so grid boundary lines are not overwritten.
      tft.fillRect(ledCenterX - 17, currentTextY, 34, tft.fontHeight(), TFT_BLACK);

      String currentText = String(currentValueRoundedUp) + "A";
      int textWidth = tft.textWidth(currentText);
      int xCoordinate = ledCenterX - (textWidth / 2);

      // Print the new value
      tft.setCursor(xCoordinate, currentTextY);
      tft.print(currentText);
      prevCurrentValues[i] = ChannelRuntime[i].CurrentValue;
    }
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
  }

  // Check for SD card status change
  if (SDCardOK != prevSDOK)
  {
    if (!SDCardOK)
    {
      PushImageRaw16BitInterruptingWrite(0, 4, ICON_WIDTH, ICON_HEIGHT, logiconError);
    }
    else
    {
      PushImageRaw16BitInterruptingWrite(0, 4, ICON_WIDTH, ICON_HEIGHT, logicon);
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
        PushImageRaw16BitInterruptingWrite(47, 4, ICON_WIDTH, ICON_HEIGHT, gpsOK);
      }
      else
      {
        PushImageRaw16BitInterruptingWrite(47, 4, ICON_WIDTH, ICON_HEIGHT, gpsError);
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
      PushImageRaw16BitInterruptingWrite(99, 4, ICON_WIDTH, ICON_HEIGHT, motion_ok);
    }
    else
    {
      PushImageRaw16BitInterruptingWrite(99, 4, ICON_WIDTH, ICON_HEIGHT, motion_error);
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
      PushImageRaw16BitInterruptingWrite(220, 4, ICON_WIDTH, ICON_HEIGHT, pc_ok);
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
      PushImageRaw16BitInterruptingWrite(151, 4, ICON_WIDTH, ICON_HEIGHT, zero_bar);
      break;
    case 1:
      PushImageRaw16BitInterruptingWrite(151, 4, ICON_WIDTH, ICON_HEIGHT, one_bar);
      break;
    case 2:
      PushImageRaw16BitInterruptingWrite(151, 4, ICON_WIDTH, ICON_HEIGHT, two_bar);
      break;
    case 3:
      PushImageRaw16BitInterruptingWrite(151, 4, ICON_WIDTH, ICON_HEIGHT, three_bar);
      break;
    case 4:
      PushImageRaw16BitInterruptingWrite(151, 4, ICON_WIDTH, ICON_HEIGHT, four_bar);
      break;
    case 5:
      PushImageRaw16BitInterruptingWrite(151, 4, ICON_WIDTH, ICON_HEIGHT, five_bar);
      break;
    }
    prevBars = bars;
  }

  int currentHour = rtc.getHours();
  int currentMinute = rtc.getMinutes();
  if (prevHour != currentHour || prevMin != currentMinute)
  {
    char timeString[6];
    snprintf(timeString, sizeof(timeString), "%02d:%02d", currentHour, currentMinute);
    tft.fillRect(271, 21, 40, 15, TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    prevHour = currentHour;
    prevMin = currentMinute;
    tft.setCursor(271, 21);
    tft.print(timeString);
  }
  tft.endWrite();
  spix.end();
}