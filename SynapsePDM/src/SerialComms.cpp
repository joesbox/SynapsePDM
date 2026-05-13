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
#include "OutputHandler.h"
#include "GSM.h"
#include "IMU.h"

ChannelConfigUnion SerialChannelData;
byte configBuffer[1000] = {0};

byte statusBuffer[4096] = {0};
int statusIndex = 0;

bool receivingConfig = false;

bool previousConnectionStatus = false;

bool pendingChannelConfigSave = false;
bool pendingSystemConfigSave = false;
bool pendingAnalogueConfigSave = false;

bool IsCortexConfigSaveActive()
{
    return receivingConfig || pendingChannelConfigSave || pendingSystemConfigSave || pendingAnalogueConfigSave;
}

uint32_t lastComms = 0;

unsigned int readBufIdx = 0;

// Keep transfer chunks at the last known stable size.
static const size_t LOG_TRANSFER_CHUNK_BYTES = 4096;
static const uint16_t LOG_BULK_MAGIC = 0x4C42;
static char logChunkTextBuffer[LOG_TRANSFER_CHUNK_BYTES] = {0};
static byte logChunkPayloadBuffer[4 + sizeof(logChunkTextBuffer)] = {0};
static byte framedPacketBuffer[sizeof(logChunkPayloadBuffer) + 16] = {0};
static byte rawBulkPacketBuffer[6 + sizeof(logChunkTextBuffer) + 4] = {0};
static bool logTransferStreaming = false;
static bool logTransferBulkStreaming = false;
static bool serialCalibrationModeEnabled = false;

static void SendFramedPacket(byte commandId, const byte *payload, uint16_t payloadLength);

struct RtcCommandPayload
{
    uint16_t Year;
    uint8_t Month;
    uint8_t Day;
    uint8_t Hour;
    uint8_t Minute;
    uint8_t Second;
};

static const uint8_t RTC_COMMAND_PAYLOAD_LENGTH = 7;

struct __attribute__((packed)) CalibrationModePayload
{
    uint8_t Enabled;
};

struct __attribute__((packed)) CalibrationOverridePayload
{
    uint8_t ChannelNumber;
    uint8_t Enabled;
};

struct __attribute__((packed)) CalibrationSetKilisPayload
{
    uint8_t ChannelNumber;
    float CurrentSenseKILIS;
};

struct __attribute__((packed)) CalibrationSampleChannelPayload
{
    float CurrentValue;
    int32_t AnalogRaw;
    float CurrentSenseKILIS;
};

struct __attribute__((packed)) ControllerTelemetryPayload
{
    float SIMModuleTemp;
    float IMUTemp;
    int32_t SystemTemperature;
    float AccelX;
    float AccelY;
    float AccelZ;
    float GyroX;
    float GyroY;
    float GyroZ;
    float MagX;
    float MagY;
    float MagZ;
    float Latitude;
    float Longitude;
    float Speed;
    float Altitude;
    float Accuracy;
    int16_t VisibleSatellites;
    int16_t UsedSatellites;
    uint16_t Year;
    uint8_t Month;
    uint8_t Day;
    uint8_t Hour;
    uint8_t Minute;
    uint8_t Second;
};

static byte calibrationSamplePayload[2 + (NUM_CHANNELS * sizeof(CalibrationSampleChannelPayload))] = {0};

static void ResetCalibrationOverrides()
{
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        ChannelRuntime[i].Override = false;
        CANChannelEnableFlags[i] = false;
    }
}

static void SetCalibrationModeEnabled(bool enabled)
{
    serialCalibrationModeEnabled = enabled;
    ResetCalibrationOverrides();
    OutputsOff();
}

static bool IsValidCalibrationChannelNumber(uint8_t channelNumber)
{
    return channelNumber >= 1 && channelNumber <= NUM_CHANNELS;
}

static void SendCalibrationSamplePacket()
{
    uint16_t writeIndex = 0;
    calibrationSamplePayload[writeIndex++] = serialCalibrationModeEnabled ? 1 : 0;
    calibrationSamplePayload[writeIndex++] = NUM_CHANNELS;

    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        CalibrationSampleChannelPayload sample = {
            ChannelRuntime[i].CurrentValue,
            ChannelRuntime[i].AnalogRaw,
            Channels[i].CurrentSenseKILIS};
        memcpy(&calibrationSamplePayload[writeIndex], &sample, sizeof(sample));
        writeIndex += sizeof(sample);
    }

    SendFramedPacket(COMMAND_ID_CALIBRATION_SAMPLE, calibrationSamplePayload, writeIndex);
}

static void SendControllerTelemetryPacket()
{
    ControllerTelemetryPayload payload = {
        simModuleTemp,
        imuTemp,
        SystemRuntimeParams.SystemTemperature,
        accelX,
        accelY,
        accelZ,
        gyroX,
        gyroY,
        gyroZ,
        magX,
        magY,
        magZ,
        lat,
        lon,
        speed,
        alt,
        accuracy,
        (int16_t)vsat,
        (int16_t)usat,
        (uint16_t)year,
        (uint8_t)month,
        (uint8_t)day,
        (uint8_t)hour,
        (uint8_t)minute,
        (uint8_t)second};

    SendFramedPacket(COMMAND_ID_CONTROLLER_TELEMETRY, (const byte *)&payload, sizeof(payload));
}

static bool ApplyCalibrationOverride(const CalibrationOverridePayload &payload)
{
    if (!IsValidCalibrationChannelNumber(payload.ChannelNumber))
    {
        return false;
    }

    uint8_t channelIndex = payload.ChannelNumber - 1;
    ChannelRuntime[channelIndex].Override = payload.Enabled ? 1 : 0;
    CANChannelEnableFlags[channelIndex] = false;
    ChannelRuntime[channelIndex].ErrorFlags = 0;

    if (payload.Enabled)
    {
        enabledTimers[channelIndex] = millis();
    }
    else
    {
        ChannelRuntime[channelIndex].Enabled = false;
        ChannelRuntime[channelIndex].CurrentValue = 0.0f;
    }

    return true;
}

static bool ApplyCalibrationKilis(const CalibrationSetKilisPayload &payload)
{
    if (!IsValidCalibrationChannelNumber(payload.ChannelNumber))
    {
        return false;
    }

    if (!(payload.CurrentSenseKILIS >= MIN_CHANNEL_CURRENT_SENSE_KILIS &&
          payload.CurrentSenseKILIS <= MAX_CHANNEL_CURRENT_SENSE_KILIS))
    {
        return false;
    }

    uint8_t channelIndex = payload.ChannelNumber - 1;
    Channels[channelIndex].CurrentSenseKILIS = payload.CurrentSenseKILIS;
    pendingChannelConfigSave = true;
    saveEEPROMOnTimeout = true;
    EEPROMSaveTimout = millis() + EEPROM_WRITE_DELAY;
    return true;
}

static bool FactoryResetController()
{
    OutputsOff();
    CleanEEPROM();

    InitialiseChannelData();
    SaveChannelConfig();
    ChannelCRCValid = LoadChannelConfig();

    InitialiseSystemData();
    SaveSystemConfig();
    SystemCRCValid = LoadSystemConfig();

    InitialiseStorageData();
    SaveStorageConfig();
    StorageCRCValid = LoadStorageConfig();

    InitialiseAnalogueData();
    SaveAnalogueConfig();
    AnalogueCRCValid = LoadAnalogueConfig();

    InitialiseInputs();
    backgroundDrawn = false;
    invalidateDisplay = true;

    pendingChannelConfigSave = false;
    pendingSystemConfigSave = false;
    pendingAnalogueConfigSave = false;

    return ChannelCRCValid && SystemCRCValid && StorageCRCValid && AnalogueCRCValid;
}

static void AppendStatusByte(byte value, uint32_t &checksum)
{
    statusBuffer[statusIndex++] = value;
    checksum += value;
}

static void AppendStatusBytes(const void *data, size_t length, uint32_t &checksum)
{
    const byte *bytes = (const byte *)data;
    for (size_t i = 0; i < length; i++)
    {
        AppendStatusByte(bytes[i], checksum);
    }
}

static void BeginStatusPacket(byte commandId, uint32_t &checksum)
{
    checksum = 0;
    statusIndex = 0;
    memset(statusBuffer, 0, sizeof(statusBuffer));

    AppendStatusByte(SERIAL_HEADER & 0xFF, checksum);
    AppendStatusByte(SERIAL_HEADER >> 8, checksum);
    AppendStatusByte(commandId, checksum);
}

static void FinalizeStatusPacket(uint32_t checksum)
{
    AppendStatusByte(SERIAL_TRAILER & 0xFF, checksum);
    AppendStatusByte(SERIAL_TRAILER >> 8, checksum);

    byte checksumBytes[4] = {0};
    memcpy(checksumBytes, &checksum, sizeof(checksum));
    for (size_t i = 0; i < sizeof(checksumBytes); i++)
    {
        statusBuffer[statusIndex++] = checksumBytes[i];
    }

    Serial.write(statusBuffer, statusIndex);
}

static void SendLiveStatusPacket()
{
    uint32_t checksum = 0;
    BeginStatusPacket(COMMAND_ID_REQUEST, checksum);

    AppendStatusByte(NUM_CHANNELS, checksum);
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        AppendStatusByte((byte)Channels[i].ChanType, checksum);
        AppendStatusByte(ChannelRuntime[i].Override, checksum);
        AppendStatusBytes(&ChannelRuntime[i].CurrentValue, sizeof(ChannelRuntime[i].CurrentValue), checksum);
        AppendStatusByte(IsChannelEffectivelyEnabled(i) ? 1 : 0, checksum);
        AppendStatusByte(ChannelRuntime[i].ErrorFlags, checksum);
        AppendStatusByte(Channels[i].PWMSetDuty, checksum);
    }

    AppendStatusByte(NUM_ANA_CHANNELS, checksum);
    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        AppendStatusBytes(&AnalogueIns[i].InputVoltage, sizeof(AnalogueIns[i].InputVoltage), checksum);
        AppendStatusBytes(&AnalogueIns[i].InputValue, sizeof(AnalogueIns[i].InputValue), checksum);
    }

    AppendStatusBytes(&SystemRuntimeParams.SystemTemperature, sizeof(SystemRuntimeParams.SystemTemperature), checksum);
    AppendStatusBytes(&SystemRuntimeParams.SIMModuleTemp, sizeof(SystemRuntimeParams.SIMModuleTemp), checksum);
    AppendStatusBytes(&SystemRuntimeParams.IMUTemp, sizeof(SystemRuntimeParams.IMUTemp), checksum);
    AppendStatusBytes(&SystemRuntimeParams.VBatt, sizeof(SystemRuntimeParams.VBatt), checksum);
    AppendStatusBytes(&SystemRuntimeParams.SystemCurrent, sizeof(SystemRuntimeParams.SystemCurrent), checksum);
    AppendStatusBytes(&SystemRuntimeParams.ErrorFlags, sizeof(SystemRuntimeParams.ErrorFlags), checksum);

    uint8_t mobileSignalBars = csq_to_bars();
    AppendStatusByte(mobileSignalBars, checksum);

    FinalizeStatusPacket(checksum);
}

static void SendStaticStatusPacket()
{
    uint32_t checksum = 0;
    BeginStatusPacket(COMMAND_ID_REQUEST_STATIC, checksum);

    AppendStatusByte(NUM_CHANNELS, checksum);
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        AppendStatusByte((byte)Channels[i].ChanType, checksum);
        AppendStatusByte(ChannelRuntime[i].Override, checksum);
        AppendStatusByte(Channels[i].CurrentSensePin, checksum);
        AppendStatusBytes(&Channels[i].CurrentThresholdHigh, sizeof(Channels[i].CurrentThresholdHigh), checksum);
        AppendStatusBytes(&Channels[i].CurrentThresholdLow, sizeof(Channels[i].CurrentThresholdLow), checksum);
        AppendStatusBytes(&ChannelRuntime[i].CurrentValue, sizeof(ChannelRuntime[i].CurrentValue), checksum);
        AppendStatusByte(Channels[i].Enabled, checksum);
        AppendStatusByte(ChannelRuntime[i].ErrorFlags, checksum);
        AppendStatusByte(Channels[i].GroupNumber, checksum);
        AppendStatusByte(Channels[i].InputControlPin, checksum);
        AppendStatusBytes(&Channels[i].OnThreshold, sizeof(Channels[i].OnThreshold), checksum);
        AppendStatusBytes(&Channels[i].OffThreshold, sizeof(Channels[i].OffThreshold), checksum);
        AppendStatusBytes(&Channels[i].ScaleMin, sizeof(Channels[i].ScaleMin), checksum);
        AppendStatusBytes(&Channels[i].ScaleMax, sizeof(Channels[i].ScaleMax), checksum);
        AppendStatusByte(Channels[i].PWMMin, checksum);
        AppendStatusByte(Channels[i].PWMMax, checksum);
        AppendStatusByte(Channels[i].MultiChannel, checksum);
        AppendStatusByte(Channels[i].RetryCount, checksum);
        AppendStatusBytes(&Channels[i].InrushDelay, sizeof(Channels[i].InrushDelay), checksum);
        AppendStatusBytes(&Channels[i].ChannelName, sizeof(Channels[i].ChannelName), checksum);
        AppendStatusByte(Channels[i].RunOn, checksum);
        AppendStatusBytes(&Channels[i].RunOnTime, sizeof(Channels[i].RunOnTime), checksum);
        AppendStatusByte(Channels[i].SoftStart, checksum);
        AppendStatusBytes(&Channels[i].SoftStartTime, sizeof(Channels[i].SoftStartTime), checksum);
        AppendStatusBytes(&Channels[i].InrushCurrentThreshold, sizeof(Channels[i].InrushCurrentThreshold), checksum);
        AppendStatusByte(Channels[i].PWMSetDuty, checksum);
        AppendStatusByte(Channels[i].SoftStop, checksum);
        AppendStatusBytes(&Channels[i].SoftStopTime, sizeof(Channels[i].SoftStopTime), checksum);
        AppendStatusByte((byte)Channels[i].Category, checksum);
        AppendStatusBytes(&Channels[i].IntermittentOnTime, sizeof(Channels[i].IntermittentOnTime), checksum);
        AppendStatusBytes(&Channels[i].IntermittentOffTime, sizeof(Channels[i].IntermittentOffTime), checksum);
    }

    AppendStatusByte(NUM_ANA_CHANNELS, checksum);
    for (int i = 0; i < NUM_ANA_CHANNELS; i++)
    {
        AppendStatusByte((byte)AnalogueIns[i].ChanType, checksum);
        AppendStatusByte(AnalogueIns[i].Units, checksum);
        AppendStatusByte(AnalogueIns[i].CalibrationPoints, checksum);
        AppendStatusByte(AnalogueIns[i].PullUpEnable, checksum);
        AppendStatusByte(AnalogueIns[i].PullDownEnable, checksum);
        AppendStatusBytes(&AnalogueIns[i].InputVoltage, sizeof(AnalogueIns[i].InputVoltage), checksum);
        AppendStatusBytes(&AnalogueIns[i].InputValue, sizeof(AnalogueIns[i].InputValue), checksum);
        AppendStatusBytes(&AnalogueIns[i].CalibrationVolt1, sizeof(AnalogueIns[i].CalibrationVolt1), checksum);
        AppendStatusBytes(&AnalogueIns[i].CalibrationValue1, sizeof(AnalogueIns[i].CalibrationValue1), checksum);
        AppendStatusBytes(&AnalogueIns[i].CalibrationVolt2, sizeof(AnalogueIns[i].CalibrationVolt2), checksum);
        AppendStatusBytes(&AnalogueIns[i].CalibrationValue2, sizeof(AnalogueIns[i].CalibrationValue2), checksum);
        AppendStatusBytes(&AnalogueIns[i].CalibrationVolt3, sizeof(AnalogueIns[i].CalibrationVolt3), checksum);
        AppendStatusBytes(&AnalogueIns[i].CalibrationValue3, sizeof(AnalogueIns[i].CalibrationValue3), checksum);
        AppendStatusBytes(&AnalogueIns[i].NTCBeta, sizeof(AnalogueIns[i].NTCBeta), checksum);
        AppendStatusBytes(&AnalogueIns[i].NTCNominalResistance, sizeof(AnalogueIns[i].NTCNominalResistance), checksum);
    }

    AppendStatusBytes(&SystemRuntimeParams.SystemTemperature, sizeof(SystemRuntimeParams.SystemTemperature), checksum);
    AppendStatusBytes(&SystemRuntimeParams.SIMModuleTemp, sizeof(SystemRuntimeParams.SIMModuleTemp), checksum);
    AppendStatusBytes(&SystemRuntimeParams.IMUTemp, sizeof(SystemRuntimeParams.IMUTemp), checksum);
    AppendStatusByte(SystemParams.CANResEnabled, checksum);
    AppendStatusBytes(&SystemRuntimeParams.VBatt, sizeof(SystemRuntimeParams.VBatt), checksum);
    AppendStatusBytes(&SystemRuntimeParams.SystemCurrent, sizeof(SystemRuntimeParams.SystemCurrent), checksum);
    AppendStatusByte(SystemParams.SystemCurrentLimit, checksum);
    AppendStatusBytes(&SystemRuntimeParams.ErrorFlags, sizeof(SystemRuntimeParams.ErrorFlags), checksum);
    AppendStatusBytes(&SystemParams.ChannelDataCANID, sizeof(SystemParams.ChannelDataCANID), checksum);
    AppendStatusBytes(&SystemParams.DigitalInputDataCANID, sizeof(SystemParams.DigitalInputDataCANID), checksum);
    AppendStatusBytes(&SystemParams.AnalogueInputDataCANID, sizeof(SystemParams.AnalogueInputDataCANID), checksum);
    AppendStatusBytes(&SystemParams.SystemDataCANID, sizeof(SystemParams.SystemDataCANID), checksum);
    AppendStatusBytes(&SystemParams.ChannelConfigDataCANID, sizeof(SystemParams.ChannelConfigDataCANID), checksum);
    AppendStatusBytes(&SystemParams.SystemConfigDataCANID, sizeof(SystemParams.SystemConfigDataCANID), checksum);
    AppendStatusBytes(&SystemParams.IMUwakeWindow, sizeof(SystemParams.IMUwakeWindow), checksum);
    AppendStatusByte(SystemParams.SpeedUnitPref, checksum);
    AppendStatusByte(SystemParams.DistanceUnitPref, checksum);
    AppendStatusByte(SystemParams.AllowData, checksum);
    AppendStatusByte(SystemParams.AllowGPS, checksum);
    AppendStatusByte(SystemParams.AllowMotionDetect, checksum);

    uint8_t mobileSignalBars = csq_to_bars();
    AppendStatusByte(mobileSignalBars, checksum);
    AppendStatusBytes(&SystemParams.TimeZone, sizeof(SystemParams.TimeZone), checksum);

    FinalizeStatusPacket(checksum);
}

static bool WaitForSerialBytes(uint16_t requiredBytes, uint32_t timeoutMs)
{
    uint32_t start = millis();
    while (Serial.available() < requiredBytes)
    {
        if ((millis() - start) >= timeoutMs)
        {
            return false;
        }

        IWatchdog.reload();
        delay(1);
    }

    return true;
}

static bool ReadSerialBytesExact(uint8_t *buffer, uint16_t length, uint32_t timeoutMs)
{
    if (buffer == nullptr)
    {
        return false;
    }

    uint32_t start = millis();
    uint16_t bytesRead = 0;
    while (bytesRead < length)
    {
        while (Serial.available() > 0 && bytesRead < length)
        {
            buffer[bytesRead++] = (uint8_t)Serial.read();
            start = millis();
        }

        if (bytesRead >= length)
        {
            return true;
        }

        if ((millis() - start) >= timeoutMs)
        {
            return false;
        }

        IWatchdog.reload();
        delay(1);
    }

    return true;
}

static bool WriteSerialBytes(const byte *buffer, size_t length)
{
    size_t bytesRemaining = length;
    size_t offset = 0;
    while (bytesRemaining > 0)
    {
        size_t written = Serial.write(&buffer[offset], bytesRemaining);
        if (written == 0)
        {
            IWatchdog.reload();
            delayMicroseconds(100);
            continue;
        }

        offset += written;
        bytesRemaining -= written;
        lastComms = millis();
        pcCommsOK = true;
        IWatchdog.reload();
    }

    return true;
}

static bool ReadRtcCommandPayload(RtcCommandPayload *payload)
{
    if (payload == nullptr)
    {
        return false;
    }

    uint8_t buffer[RTC_COMMAND_PAYLOAD_LENGTH] = {0};
    if (!ReadSerialBytesExact(buffer, RTC_COMMAND_PAYLOAD_LENGTH, 2000))
    {
        return false;
    }

    payload->Year = (uint16_t)buffer[0] | (uint16_t)(buffer[1] << 8);
    payload->Month = buffer[2];
    payload->Day = buffer[3];
    payload->Hour = buffer[4];
    payload->Minute = buffer[5];
    payload->Second = buffer[6];

    return true;
}

static bool SendRawBulkLogPacket(const char *chunkText, uint16_t chunkBytes, uint8_t progress, bool done)
{
    uint32_t checksum = 0;
    uint16_t writeIndex = 0;

    byte send = LOG_BULK_MAGIC & 0xFF;
    rawBulkPacketBuffer[writeIndex++] = send;
    checksum += send;

    send = LOG_BULK_MAGIC >> 8;
    rawBulkPacketBuffer[writeIndex++] = send;
    checksum += send;

    rawBulkPacketBuffer[writeIndex++] = progress;
    checksum += progress;

    send = done ? 1 : 0;
    rawBulkPacketBuffer[writeIndex++] = send;
    checksum += send;

    send = (byte)(chunkBytes & 0xFF);
    rawBulkPacketBuffer[writeIndex++] = send;
    checksum += send;

    send = (byte)((chunkBytes >> 8) & 0xFF);
    rawBulkPacketBuffer[writeIndex++] = send;
    checksum += send;

    for (uint16_t i = 0; i < chunkBytes; i++)
    {
        send = (byte)chunkText[i];
        rawBulkPacketBuffer[writeIndex++] = send;
        checksum += send;
    }

    rawBulkPacketBuffer[writeIndex++] = (byte)(checksum & 0xFF);
    rawBulkPacketBuffer[writeIndex++] = (byte)((checksum >> 8) & 0xFF);
    rawBulkPacketBuffer[writeIndex++] = (byte)((checksum >> 16) & 0xFF);
    rawBulkPacketBuffer[writeIndex++] = (byte)((checksum >> 24) & 0xFF);

    return WriteSerialBytes(rawBulkPacketBuffer, writeIndex);
}

static bool SendNextRawBulkLogChunk()
{
    uint16_t chunkBytes = 0;
    uint8_t progress = 0;
    bool done = false;

    if (!ReadLogTransferChunk(logChunkTextBuffer, sizeof(logChunkTextBuffer), 0, &chunkBytes, &progress, &done))
    {
        return false;
    }

    if (!SendRawBulkLogPacket(logChunkTextBuffer, chunkBytes, progress, done))
    {
        return false;
    }

    logTransferBulkStreaming = !done;
    return true;
}

static void SendFramedPacket(byte commandId, const byte *payload, uint16_t payloadLength)
{
    uint32_t checksum = 0;

    // Keep comms status alive while large frames are emitted.
    lastComms = millis();
    pcCommsOK = true;

    uint16_t frameLength = (uint16_t)(payloadLength + 9);
    if (frameLength > sizeof(framedPacketBuffer))
    {
        // Fallback path should never happen with current payload sizing.
        return;
    }

    uint16_t writeIndex = 0;
    byte send = SERIAL_HEADER & 0xFF;
    framedPacketBuffer[writeIndex++] = send;
    checksum += send;

    send = SERIAL_HEADER >> 8;
    framedPacketBuffer[writeIndex++] = send;
    checksum += send;

    framedPacketBuffer[writeIndex++] = commandId;
    checksum += commandId;

    for (uint16_t i = 0; i < payloadLength; i++)
    {
        framedPacketBuffer[writeIndex++] = payload[i];
        checksum += payload[i];
    }

    send = SERIAL_TRAILER & 0xFF;
    framedPacketBuffer[writeIndex++] = send;
    checksum += send;

    send = SERIAL_TRAILER >> 8;
    framedPacketBuffer[writeIndex++] = send;
    checksum += send;

    framedPacketBuffer[writeIndex++] = (byte)(checksum & 0xFF);
    framedPacketBuffer[writeIndex++] = (byte)((checksum >> 8) & 0xFF);
    framedPacketBuffer[writeIndex++] = (byte)((checksum >> 16) & 0xFF);
    framedPacketBuffer[writeIndex++] = (byte)((checksum >> 24) & 0xFF);

    WriteSerialBytes(framedPacketBuffer, writeIndex);
}

static bool SendNextLogTransferChunk()
{
    uint16_t chunkBytes = 0;
    uint8_t progress = 0;
    bool done = false;

    if (!ReadLogTransferChunk(logChunkTextBuffer, sizeof(logChunkTextBuffer), 0, &chunkBytes, &progress, &done))
    {
        return false;
    }

    logChunkPayloadBuffer[0] = progress;
    logChunkPayloadBuffer[1] = done ? 1 : 0;
    logChunkPayloadBuffer[2] = (byte)(chunkBytes & 0xFF);
    logChunkPayloadBuffer[3] = (byte)((chunkBytes >> 8) & 0xFF);
    if (chunkBytes > 0)
    {
        memcpy(&logChunkPayloadBuffer[4], logChunkTextBuffer, chunkBytes);
    }

    SendFramedPacket(COMMAND_ID_LOG_CHUNK, logChunkPayloadBuffer, 4 + chunkBytes);
    lastComms = millis();
    pcCommsOK = true;

    return true;
}

static bool IsTemperatureUnits(uint8_t units)
{
    return units == ANA_UNITS_CELSIUS || units == ANA_UNITS_FAHRENHEIT;
}

static float ConvertTemperatureValue(float value, uint8_t fromUnits, uint8_t toUnits)
{
    if (fromUnits == toUnits)
    {
        return value;
    }

    if (fromUnits == ANA_UNITS_CELSIUS && toUnits == ANA_UNITS_FAHRENHEIT)
    {
        return (value * 1.8f) + 32.0f;
    }

    if (fromUnits == ANA_UNITS_FAHRENHEIT && toUnits == ANA_UNITS_CELSIUS)
    {
        return (value - 32.0f) / 1.8f;
    }

    return value;
}

static void SanitizeAnalogueConfigForType(AnalogueInputs &input)
{
    if (input.CalibrationPoints < 2)
    {
        input.CalibrationPoints = 2;
    }
    if (input.CalibrationPoints > 3)
    {
        input.CalibrationPoints = 3;
    }

    if (input.NTCBeta < 1.0f)
    {
        input.NTCBeta = 3950.0f;
    }

    if (input.NTCNominalResistance < 1.0f)
    {
        input.NTCNominalResistance = 10000.0f;
    }

    switch (input.ChanType)
    {
    case RAW_VOLTAGE:
        input.PullUpEnable = false;
        input.PullDownEnable = false;
        input.Units = ANA_UNITS_VOLTS;
        break;
    case ACTIVE:
        input.PullUpEnable = false;
        input.PullDownEnable = false;
        break;
    case PASSIVE:
        if (!input.PullUpEnable && !input.PullDownEnable)
        {
            input.PullUpEnable = true;
        }
        if (input.PullUpEnable && input.PullDownEnable)
        {
            input.PullDownEnable = false;
        }
        break;
    case NTC:
        if (!input.PullUpEnable && !input.PullDownEnable)
        {
            input.PullUpEnable = true;
        }
        if (input.PullUpEnable && input.PullDownEnable)
        {
            input.PullDownEnable = false;
        }
        if (!IsTemperatureUnits(input.Units))
        {
            input.Units = ANA_UNITS_CELSIUS;
        }
        break;
    case DIGITAL:
        if (!input.PullUpEnable && !input.PullDownEnable)
        {
            input.PullDownEnable = true;
        }
        if (input.PullUpEnable && input.PullDownEnable)
        {
            input.PullDownEnable = false;
        }
        input.Units = ANA_UNITS_VOLTS;
        break;
    default:
        input.ChanType = RAW_VOLTAGE;
        input.PullUpEnable = false;
        input.PullDownEnable = false;
        input.Units = ANA_UNITS_VOLTS;
        break;
    }
}

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
}

void CheckSerial()
{
    if (previousConnectionStatus != pcCommsOK)
    {
        previousConnectionStatus = pcCommsOK;
        if (pcCommsOK)
        {
            IWatchdog.set(10000 * 1000); // 5 second watchdog when PC comms active (microseconds)
        }
        else
        {
            IWatchdog.set(2000 * 1000); // 2 second watchdog when PC comms inactive (microseconds)
        }
    }
    if ((millis() - lastComms > COMMS_TIMEOUT))
    {
        connectionStatus = 0; // Timeout - reset connection
        pcCommsOK = false;
        receivingConfig = false;
        logTransferStreaming = false;
        logTransferBulkStreaming = false;
        serialCalibrationModeEnabled = false;
        CancelLogTransfer();
        readBufIdx = 0;
        recBytesRead = 0;
        for (int i = 0; i < NUM_CHANNELS; i++)
        {
            ChannelRuntime[i].Override = false; // Clear any overrides
        }
    }
    while (Serial.available() && !receivingConfig)
    {
        pcCommsOK = true;
        lastComms = millis();
        byte nextByte = Serial.read();
        switch (nextByte)
        {
        case COMMAND_ID_BEGIN:
            Serial.write(COMMAND_ID_CONFIM);
            break;

        case COMMAND_ID_SKIP:
            Serial.write(COMMAND_ID_CONFIM);
            connectionStatus = 1;
            break;

        case COMMAND_ID_REQUEST:
        {
            SendLiveStatusPacket();

            break;
        }

        case COMMAND_ID_REQUEST_STATIC:
        {
            SendStaticStatusPacket();

            break;
        }
        case COMMAND_ID_NEWCONFIG:
        {
            connectionStatus = 2;
            // New config incoming, read all bytes into buffer
            uint32_t calcChecksum = 0;
            bool validPacket = false;
            bool inputConfigChanged = false;
            receivingConfig = true;
            unsigned long firstByteDeadline = millis() + 30;
            unsigned long settleDeadline = 0;

            while (millis() <= firstByteDeadline || settleDeadline != 0)
            {
                if (Serial.available())
                {
                    connectionStatus = 3; // Receiving config data
                    while (Serial.available())
                    {
                        if (readBufIdx < 1000)
                        {
                            configBuffer[readBufIdx] = Serial.read();
                            readBufIdx++;
                            recBytesRead = readBufIdx;
                        }
                        else
                        {
                            Serial.read(); // Overflow - discard byte
                        }
                    }
                    settleDeadline = millis() + 4;
                }
                else if (settleDeadline != 0 && millis() > settleDeadline)
                {
                    break;
                }
            }

            receivingConfig = false;

            if (readBufIdx < 8)
            {
                validPacket = false;
                connectionStatus = 9;
            }
            else
            {

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
                            pendingChannelConfigSave = true;
                            connectionStatus = 4;
                            switch (configBuffer[CONFIG_PARAMETER_INDEX])
                            {
                            case 0: // Channel type
                            {
                                uint8_t channelIndex = configBuffer[CONFIG_DATA_INDEX];
                                ChannelType previousType = Channels[channelIndex].ChanType;
                                Channels[channelIndex].ChanType = (ChannelType)configBuffer[CONFIG_DATA_START_INDEX];
                                if (previousType != DIG_INTERMITTENT && Channels[channelIndex].ChanType == DIG_INTERMITTENT)
                                {
                                    Channels[channelIndex].IntermittentOnTime = 1000;
                                    Channels[channelIndex].IntermittentOffTime = 1000;
                                }
                                if (SyncChannelTypeForAssignedInput(channelIndex))
                                {
                                    pendingChannelConfigSave = true;
                                }
                                inputConfigChanged = true;
                                break;
                            }
                            case 1: // Override flag
                                ChannelRuntime[configBuffer[CONFIG_DATA_INDEX]].Override = configBuffer[CONFIG_DATA_START_INDEX];
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
                                if (SyncChannelTypeForAssignedInput(configBuffer[CONFIG_DATA_INDEX]))
                                {
                                    pendingChannelConfigSave = true;
                                }
                                inputConfigChanged = true;
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
                            case 13: // Soft start enable
                                Channels[configBuffer[CONFIG_DATA_INDEX]].SoftStart = configBuffer[CONFIG_DATA_START_INDEX];
                                break;
                            case 14: // Soft start time
                                memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].SoftStartTime, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].SoftStartTime));
                                if (Channels[configBuffer[CONFIG_DATA_INDEX]].SoftStartTime > MAX_SOFT_START_TIME)
                                {
                                    Channels[configBuffer[CONFIG_DATA_INDEX]].SoftStartTime = MAX_SOFT_START_TIME;
                                }
                                break;
                            case 15: // Inrush current threshold
                                memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].InrushCurrentThreshold, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].InrushCurrentThreshold));
                                if (Channels[configBuffer[CONFIG_DATA_INDEX]].InrushCurrentThreshold > INRUSH_CURRENT_MAX)
                                {
                                    Channels[configBuffer[CONFIG_DATA_INDEX]].InrushCurrentThreshold = INRUSH_CURRENT_MAX;
                                }
                                if (Channels[configBuffer[CONFIG_DATA_INDEX]].InrushCurrentThreshold < 0)
                                {
                                    Channels[configBuffer[CONFIG_DATA_INDEX]].InrushCurrentThreshold = 0;
                                }
                                break;
                            case 16: // PWM set duty
                                Channels[configBuffer[CONFIG_DATA_INDEX]].PWMSetDuty = configBuffer[CONFIG_DATA_START_INDEX];
                                break;
                            case 17: // On threshold
                                memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].OnThreshold, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].OnThreshold));
                                break;
                            case 18: // Off threshold
                                memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].OffThreshold, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].OffThreshold));
                                break;
                            case 19: // Scale min
                                memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].ScaleMin, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].ScaleMin));
                                break;
                            case 20: // Scale max
                                memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].ScaleMax, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].ScaleMax));
                                break;
                            case 21: // PWM min
                                Channels[configBuffer[CONFIG_DATA_INDEX]].PWMMin = configBuffer[CONFIG_DATA_START_INDEX];
                                break;
                            case 22: // PWM max
                                Channels[configBuffer[CONFIG_DATA_INDEX]].PWMMax = configBuffer[CONFIG_DATA_START_INDEX];
                                break;
                            case 23: // Soft stop enable
                                Channels[configBuffer[CONFIG_DATA_INDEX]].SoftStop = configBuffer[CONFIG_DATA_START_INDEX];
                                break;
                            case 24: // Soft stop time
                                memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].SoftStopTime, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].SoftStopTime));
                                if (Channels[configBuffer[CONFIG_DATA_INDEX]].SoftStopTime > MAX_SOFT_STOP_TIME)
                                {
                                    Channels[configBuffer[CONFIG_DATA_INDEX]].SoftStopTime = MAX_SOFT_STOP_TIME;
                                }
                                break;
                            case 25: // Output category
                                Channels[configBuffer[CONFIG_DATA_INDEX]].Category = SanitizeChannelCategory(configBuffer[CONFIG_DATA_START_INDEX]);
                                break;
                            case 26: // Intermittent on time
                                memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].IntermittentOnTime, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].IntermittentOnTime));
                                if (Channels[configBuffer[CONFIG_DATA_INDEX]].IntermittentOnTime > MAX_INTERMITTENT_TIME)
                                {
                                    Channels[configBuffer[CONFIG_DATA_INDEX]].IntermittentOnTime = MAX_INTERMITTENT_TIME;
                                }
                                break;
                            case 27: // Intermittent off time
                                memcpy(&Channels[configBuffer[CONFIG_DATA_INDEX]].IntermittentOffTime, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(Channels[configBuffer[CONFIG_DATA_INDEX]].IntermittentOffTime));
                                if (Channels[configBuffer[CONFIG_DATA_INDEX]].IntermittentOffTime > MAX_INTERMITTENT_TIME)
                                {
                                    Channels[configBuffer[CONFIG_DATA_INDEX]].IntermittentOffTime = MAX_INTERMITTENT_TIME;
                                }
                                break;

                            default:
                                // Channel parameter out of range. Ignore packet
                                validPacket = false;
                                break;
                            }

                            break;
                        case CONFIG_DATA_ANALOGUE:
                            pendingAnalogueConfigSave = true;
                            switch (configBuffer[CONFIG_PARAMETER_INDEX])
                            {
                            case 0: // Pull-up enable
                                AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].PullUpEnable = configBuffer[CONFIG_DATA_START_INDEX];
                                inputConfigChanged = true;
                                break;
                            case 1: // Pull-down enable
                                AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].PullDownEnable = configBuffer[CONFIG_DATA_START_INDEX];
                                inputConfigChanged = true;
                                break;
                            case 2: // Channel type
                            {
                                uint8_t inputIndex = configBuffer[CONFIG_DATA_INDEX];
                                AnalogueIns[inputIndex].ChanType = (AnalogueChannelType)configBuffer[CONFIG_DATA_START_INDEX];

                                if (AnalogueIns[inputIndex].ChanType == NTC)
                                {
                                    AnalogueIns[inputIndex].PullUpEnable = true;
                                    AnalogueIns[inputIndex].PullDownEnable = false;
                                    AnalogueIns[inputIndex].NTCNominalResistance = 10000.0f;
                                }

                                if (SyncChannelTypesForAnalogueInput(inputIndex))
                                {
                                    pendingChannelConfigSave = true;
                                }
                                inputConfigChanged = true;
                                break;
                            }
                            case 3: // Units
                            {
                                uint8_t inputIndex = configBuffer[CONFIG_DATA_INDEX];
                                uint8_t previousUnits = AnalogueIns[inputIndex].Units;
                                AnalogueIns[inputIndex].Units = (AnalogueUnits)configBuffer[CONFIG_DATA_START_INDEX];

                                uint8_t sanitizedUnits = AnalogueIns[inputIndex].Units;
                                if (IsTemperatureUnits(previousUnits) && IsTemperatureUnits(sanitizedUnits) && previousUnits != sanitizedUnits)
                                {
                                    uint8_t inputPin = ANAchannelInputPins[inputIndex];
                                    for (int channel = 0; channel < NUM_CHANNELS; channel++)
                                    {
                                        if (Channels[channel].InputControlPin != inputPin)
                                        {
                                            continue;
                                        }

                                        switch (Channels[channel].ChanType)
                                        {
                                        case ANA:
                                            Channels[channel].OnThreshold = ConvertTemperatureValue(Channels[channel].OnThreshold, previousUnits, sanitizedUnits);
                                            Channels[channel].OffThreshold = ConvertTemperatureValue(Channels[channel].OffThreshold, previousUnits, sanitizedUnits);
                                            break;
                                        case ANA_PWM:
                                            Channels[channel].ScaleMin = ConvertTemperatureValue(Channels[channel].ScaleMin, previousUnits, sanitizedUnits);
                                            Channels[channel].ScaleMax = ConvertTemperatureValue(Channels[channel].ScaleMax, previousUnits, sanitizedUnits);
                                            break;
                                        default:
                                            break;
                                        }
                                    }
                                }
                                inputConfigChanged = true;
                                break;
                            }
                            case 4: // Calibration points
                                AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].CalibrationPoints = configBuffer[CONFIG_DATA_START_INDEX];
                                inputConfigChanged = true;
                                break;
                            case 5: // Calibration point 1 voltage
                                memcpy(&AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].CalibrationVolt1, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].CalibrationVolt1));
                                inputConfigChanged = true;
                                break;
                            case 6: // Calibration point 1 value
                                memcpy(&AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].CalibrationValue1, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].CalibrationValue1));
                                inputConfigChanged = true;
                                break;
                            case 7: // Calibration point 2 voltage
                                memcpy(&AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].CalibrationVolt2, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].CalibrationVolt2));
                                inputConfigChanged = true;
                                break;
                            case 8: // Calibration point 2 value
                                memcpy(&AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].CalibrationValue2, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].CalibrationValue2));
                                inputConfigChanged = true;
                                break;
                            case 9: // Calibration point 3 voltage
                                memcpy(&AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].CalibrationVolt3, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].CalibrationVolt3));
                                inputConfigChanged = true;
                                break;
                            case 10: // Calibration point 3 value
                                memcpy(&AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].CalibrationValue3, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].CalibrationValue3));
                                inputConfigChanged = true;
                                break;
                            case 11: // NTC Beta
                                memcpy(&AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].NTCBeta, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].NTCBeta));
                                inputConfigChanged = true;
                                break;
                            case 12: // NTC nominal resistance at 25C
                                memcpy(&AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].NTCNominalResistance, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(AnalogueIns[configBuffer[CONFIG_DATA_INDEX]].NTCNominalResistance));
                                inputConfigChanged = true;
                                break;
                            default:
                                // Analogue parameter out of range. Ignore packet
                                validPacket = false;
                                break;
                            }

                            if (validPacket && configBuffer[CONFIG_PARAMETER_INDEX] > 1)
                            {
                                SanitizeAnalogueConfigForType(AnalogueIns[configBuffer[CONFIG_DATA_INDEX]]);
                            }

                            connectionStatus = 5;

                            break;

                        case CONFIG_DATA_SYSTEM:
                            pendingSystemConfigSave = true;

                            switch (configBuffer[CONFIG_PARAMETER_INDEX])
                            {
                            case 0: // CAN resistor enable
                                SystemParams.CANResEnabled = configBuffer[CONFIG_DATA_START_INDEX];
                                break;
                            case 1: // Channel CAN data ID
                                memcpy(&SystemParams.ChannelDataCANID, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(SystemParams.ChannelDataCANID));
                                break;
                            case 2: // Digital input CAN data ID
                                memcpy(&SystemParams.DigitalInputDataCANID, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(SystemParams.DigitalInputDataCANID));
                                break;
                            case 3: // Analogue input CAN data ID
                                memcpy(&SystemParams.AnalogueInputDataCANID, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(SystemParams.AnalogueInputDataCANID));
                                break;
                            case 4: // System CAN ID
                                memcpy(&SystemParams.SystemDataCANID, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(SystemParams.SystemDataCANID));
                                break;
                            case 5: // Config CAN ID
                                memcpy(&SystemParams.ChannelConfigDataCANID, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(SystemParams.ChannelConfigDataCANID));
                                break;
                            case 6: // IMU wake window
                                memcpy(&SystemParams.IMUwakeWindow, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(SystemParams.IMUwakeWindow));
                                break;
                            case 7: // Speed unit preference
                                SystemParams.SpeedUnitPref = configBuffer[CONFIG_DATA_START_INDEX];
                                break;
                            case 8: // Distance unit preference
                                SystemParams.DistanceUnitPref = configBuffer[CONFIG_DATA_START_INDEX];
                                break;
                            case 9: // Allow mobile data
                                SystemParams.AllowData = configBuffer[CONFIG_DATA_START_INDEX];
                                break;
                            case 10: // Allow GPS
                                SystemParams.AllowGPS = configBuffer[CONFIG_DATA_START_INDEX];
                                break;
                            case 11: // Allow motion detect wake
                                SystemParams.AllowMotionDetect = configBuffer[CONFIG_DATA_START_INDEX];
                                break;
                            case 12: // System config CAN ID
                                memcpy(&SystemParams.SystemConfigDataCANID, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(SystemParams.SystemConfigDataCANID));
                                break;
                            case 13: // Time zone and DST rule blob
                                memcpy(&SystemParams.TimeZone, &configBuffer[CONFIG_DATA_START_INDEX], sizeof(SystemParams.TimeZone));
                                SanitizeTimeZoneRule(&SystemParams.TimeZone);
                                if (SystemParams.TimeZone.DSTEnabled == 0)
                                {
                                    SystemParams.DSTActive = 0;
                                }
                                break;
                            default:
                                // System parameter out of range. Ignore packet
                                validPacket = false;
                                break;
                            }
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
            }

            if (validPacket)
            {
                Serial.write(COMMAND_ID_CONFIM);
                connectionStatus = 10;
                if (inputConfigChanged)
                {
                    InitialiseInputs();
                }
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
            bool allSaved = true;
            bool anySaveRequested = pendingChannelConfigSave || pendingSystemConfigSave || pendingAnalogueConfigSave;

            if (pendingChannelConfigSave)
            {
                SaveChannelConfig();
                ChannelCRCValid = LoadChannelConfig();
                if (!ChannelCRCValid)
                {
                    allSaved &= false;
                    connectionStatus = 11;
                }
                else
                {
                    pendingChannelConfigSave = false;
                }
            }

            if (pendingSystemConfigSave)
            {
                SaveSystemConfig();
                SystemCRCValid = LoadSystemConfig();
                if (!SystemCRCValid)
                {
                    allSaved &= false;
                    connectionStatus = 12;
                }
                else
                {
                    pendingSystemConfigSave = false;
                }
            }

            if (pendingAnalogueConfigSave)
            {
                SaveAnalogueConfig();
                AnalogueCRCValid = LoadAnalogueConfig();
                if (!AnalogueCRCValid)
                {
                    allSaved &= false;
                    connectionStatus = 13;
                }
                else
                {
                    pendingAnalogueConfigSave = false;
                    InitialiseInputs();
                }
            }

            if (anySaveRequested)
            {
                invalidateDisplay = true;
            }

            Serial.write(allSaved ? COMMAND_ID_CONFIM : COMMAND_ID_CHECKSUM_FAIL);
            break;
        }
        case COMMAND_ID_FW_VER:
            SendFramedPacket(COMMAND_ID_FW_VER, (const byte *)FW_VER, strlen(FW_VER));
            break;

        case COMMAND_ID_BUILD_DATE:
            SendFramedPacket(COMMAND_ID_BUILD_DATE, (const byte *)BUILD_DATE, strlen(BUILD_DATE));
            break;

        case COMMAND_ID_FW_DIAGNOSTIC:
            SendFramedPacket(COMMAND_ID_FW_DIAGNOSTIC, (const byte *)GetFirmwareUpdateDiagnostic(), strlen(GetFirmwareUpdateDiagnostic()));
            break;

        case COMMAND_ID_SET_RTC:
        {
            RtcCommandPayload rtcPayload = {0};
            if (!ReadRtcCommandPayload(&rtcPayload))
            {
                Serial.write(COMMAND_ID_CHECKSUM_FAIL);
                break;
            }

            Serial.write(ApplyRtcDateTime(rtcPayload.Year, rtcPayload.Month, rtcPayload.Day, rtcPayload.Hour, rtcPayload.Minute, rtcPayload.Second) ? COMMAND_ID_CONFIM : COMMAND_ID_CHECKSUM_FAIL);
            break;
        }

        case COMMAND_ID_FACTORY_RESET:
            Serial.write(FactoryResetController() ? COMMAND_ID_CONFIM : COMMAND_ID_CHECKSUM_FAIL);
            break;

        case COMMAND_ID_CALIBRATION_MODE:
        {
            CalibrationModePayload payload = {0};
            if (!ReadSerialBytesExact((uint8_t *)&payload, sizeof(payload), 2000))
            {
                Serial.write(COMMAND_ID_CHECKSUM_FAIL);
                break;
            }

            SetCalibrationModeEnabled(payload.Enabled != 0);
            Serial.write(COMMAND_ID_CONFIM);
            break;
        }

        case COMMAND_ID_CALIBRATION_SAMPLE:
            if (!serialCalibrationModeEnabled)
            {
                Serial.write(COMMAND_ID_CHECKSUM_FAIL);
                break;
            }

            SendCalibrationSamplePacket();
            break;

        case COMMAND_ID_CALIBRATION_OVERRIDE:
        {
            CalibrationOverridePayload payload = {0};
            if (!ReadSerialBytesExact((uint8_t *)&payload, sizeof(payload), 2000))
            {
                Serial.write(COMMAND_ID_CHECKSUM_FAIL);
                break;
            }

            Serial.write((serialCalibrationModeEnabled && ApplyCalibrationOverride(payload)) ? COMMAND_ID_CONFIM : COMMAND_ID_CHECKSUM_FAIL);
            break;
        }

        case COMMAND_ID_CALIBRATION_SET_KILIS:
        {
            CalibrationSetKilisPayload payload = {0};
            if (!ReadSerialBytesExact((uint8_t *)&payload, sizeof(payload), 2000))
            {
                Serial.write(COMMAND_ID_CHECKSUM_FAIL);
                break;
            }

            Serial.write((serialCalibrationModeEnabled && ApplyCalibrationKilis(payload)) ? COMMAND_ID_CONFIM : COMMAND_ID_CHECKSUM_FAIL);
            break;
        }

        case COMMAND_ID_CONTROLLER_TELEMETRY:
            SendControllerTelemetryPacket();
            break;

        case COMMAND_ID_FW_UPLOAD_BEGIN:
        {
            uint8_t beginPayload[5] = {0};
            if (!ReadSerialBytesExact(beginPayload, sizeof(beginPayload), 2000))
            {
                SetFirmwareUpdateDiagnostic("upload begin payload timeout");
                Serial.write(COMMAND_ID_CHECKSUM_FAIL);
                break;
            }

            uint8_t assetType = beginPayload[0];

            Serial.write(BeginFirmwareAssetUpload(assetType) ? COMMAND_ID_CONFIM : COMMAND_ID_CHECKSUM_FAIL);
            break;
        }

        case COMMAND_ID_FW_UPLOAD_CHUNK:
        {
            if (!WaitForSerialBytes(2, 2000))
            {
                SetFirmwareUpdateDiagnostic("upload chunk header timeout");
                CancelFirmwareAssetUpload();
                Serial.write(COMMAND_ID_CHECKSUM_FAIL);
                break;
            }

            uint16_t chunkLength = (uint8_t)Serial.read();
            chunkLength |= (uint16_t)((uint8_t)Serial.read() << 8);
            if (chunkLength == 0)
            {
                SetFirmwareUpdateDiagnostic("upload chunk empty ack");
                Serial.write(COMMAND_ID_CONFIM);
                break;
            }

            uint8_t chunkBuffer[1024] = {0};
            if (chunkLength > sizeof(chunkBuffer))
            {
                SetFirmwareUpdateDiagnostic("upload chunk too large");
                CancelFirmwareAssetUpload();
                Serial.write(COMMAND_ID_CHECKSUM_FAIL);
                break;
            }

            if (!ReadSerialBytesExact(chunkBuffer, chunkLength, 5000))
            {
                SetFirmwareUpdateDiagnostic("upload chunk payload timeout");
                CancelFirmwareAssetUpload();
                Serial.write(COMMAND_ID_CHECKSUM_FAIL);
                break;
            }

            Serial.write(WriteFirmwareAssetChunk(chunkBuffer, chunkLength) ? COMMAND_ID_CONFIM : COMMAND_ID_CHECKSUM_FAIL);
            break;
        }

        case COMMAND_ID_FW_UPLOAD_END:
            Serial.write(FinishFirmwareAssetUpload() ? COMMAND_ID_CONFIM : COMMAND_ID_CHECKSUM_FAIL);
            break;

        case COMMAND_ID_FW_UPLOAD_CANCEL:
            CancelFirmwareAssetUpload();
            Serial.write(COMMAND_ID_CONFIM);
            break;

        case COMMAND_ID_FW_INSTALL:
            SetOutputsInhibited(true);
            if (InstallStagedFirmware())
            {
                Serial.write(COMMAND_ID_CONFIM);
                Serial.flush();
                delay(150);
                NVIC_SystemReset();
            }
            else
            {
                SetOutputsInhibited(false);
                Serial.write(COMMAND_ID_CHECKSUM_FAIL);
            }
            break;

        case COMMAND_ID_LOG_LIST:
        {
            lastComms = millis();
            pcCommsOK = true;

            byte count = GetAvailableLogFileCount();
            if (count > NUMBER_LOGS)
            {
                count = NUMBER_LOGS;
            }

            const byte fileNameLength = 24;
            const byte fileSizeLength = 4;
            const byte entryLength = fileNameLength + fileSizeLength;
            byte payload[1 + (NUMBER_LOGS * entryLength)] = {0};
            payload[0] = count;

            for (byte i = 0; i < count; i++)
            {
                char fileNameBuffer[fileNameLength] = {0};
                if (GetLogFileNameByIndex(i, fileNameBuffer, sizeof(fileNameBuffer)))
                {
                    uint16_t entryOffset = 1 + (i * entryLength);
                    memcpy(&payload[entryOffset], fileNameBuffer, fileNameLength);

                    uint32_t fileSizeBytes = 0;
                    if (GetLogFileSizeByIndex(i, &fileSizeBytes))
                    {
                        payload[entryOffset + fileNameLength + 0] = (byte)(fileSizeBytes & 0xFF);
                        payload[entryOffset + fileNameLength + 1] = (byte)((fileSizeBytes >> 8) & 0xFF);
                        payload[entryOffset + fileNameLength + 2] = (byte)((fileSizeBytes >> 16) & 0xFF);
                        payload[entryOffset + fileNameLength + 3] = (byte)((fileSizeBytes >> 24) & 0xFF);
                    }
                }
            }

            SendFramedPacket(COMMAND_ID_LOG_LIST, payload, sizeof(payload));
            break;
        }

        case COMMAND_ID_LOG_OPEN:
        {
            lastComms = millis();
            pcCommsOK = true;
            logTransferStreaming = false;
            logTransferBulkStreaming = false;

            uint32_t waitStart = millis();
            while (!Serial.available() && (millis() - waitStart < 750))
            {
                IWatchdog.reload();
                delay(1);
            }

            if (!Serial.available())
            {
                Serial.write(COMMAND_ID_CHECKSUM_FAIL);
                break;
            }

            uint8_t fileIndex = Serial.read();
            if (BeginLogTransfer(fileIndex))
            {
                Serial.write(COMMAND_ID_CONFIM);
            }
            else
            {
                Serial.write(COMMAND_ID_CHECKSUM_FAIL);
            }
            break;
        }

        case COMMAND_ID_LOG_CHUNK:
        {
            lastComms = millis();
            pcCommsOK = true;
            logTransferStreaming = false;
            logTransferBulkStreaming = false;

            if (!SendNextLogTransferChunk())
            {
                Serial.write(COMMAND_ID_CHECKSUM_FAIL);
            }
            break;
        }

        case COMMAND_ID_LOG_STREAM:
        {
            lastComms = millis();
            pcCommsOK = true;
            logTransferBulkStreaming = false;
            if (!logTransferStreaming)
            {
                if (!SendNextLogTransferChunk())
                {
                    Serial.write(COMMAND_ID_CHECKSUM_FAIL);
                    logTransferStreaming = false;
                    break;
                }

                logTransferStreaming = (logChunkPayloadBuffer[1] == 0);
            }

            break;
        }

        case COMMAND_ID_LOG_CANCEL:
            lastComms = millis();
            pcCommsOK = true;
            logTransferStreaming = false;
            logTransferBulkStreaming = false;
            CancelLogTransfer();
            Serial.write(COMMAND_ID_CONFIM);
            break;

        case COMMAND_ID_LOG_RESET:
            lastComms = millis();
            pcCommsOK = true;
            logTransferStreaming = false;
            logTransferBulkStreaming = false;
            Serial.write(ResetAllLogs() ? COMMAND_ID_CONFIM : COMMAND_ID_CHECKSUM_FAIL);
            break;

        case COMMAND_ID_LOG_BULK:
        {
            lastComms = millis();
            pcCommsOK = true;
            logTransferStreaming = false;

            if (!logTransferBulkStreaming)
            {
                if (!SendNextRawBulkLogChunk())
                {
                    logTransferBulkStreaming = false;
                    CancelLogTransfer();
                    break;
                }
            }

            break;
        }
        }
    }

    if (logTransferStreaming && !receivingConfig)
    {
        IWatchdog.reload();
        if (!SendNextLogTransferChunk())
        {
            logTransferStreaming = false;
            CancelLogTransfer();
            Serial.write(COMMAND_ID_CHECKSUM_FAIL);
        }
    }

    if (logTransferBulkStreaming && !receivingConfig)
    {
        IWatchdog.reload();
        if (!SendNextRawBulkLogChunk())
        {
            logTransferBulkStreaming = false;
            CancelLogTransfer();
        }
    }
}
