/*  GSM.cpp GSM and GPS variables, functions and data handling.
    Copyright (c) 2025 Joe Mann.  All right reserved.

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

#include "GSM.h"
#include "System.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>

// #define DEBUG

uint32_t lastGPSTime = 0;

extern bool CellularCRCValid;

static const char INTERNET_PROBE_HOST[] = "www.google.com";

static bool hasAcceptedGPSFix = false;
static bool hasSeenGNSSResponse = false;
static float lastAcceptedLat = 0.0f;
static float lastAcceptedLon = 0.0f;
static float lastAcceptedSpeedKnots = 0.0f;
static uint32_t lastGNSSFixSeenAt = 0;
static char lastGNSSInfoResponse[128] = {0};
static char lastGPSParseStatus[80] = "No GNSS response parsed.";

static bool previousGPSEnable = false;
static int rssi = SIM7600_CSQ_RSSI_UNKNOWN;
static uint8_t SIM7600State = 0; // 0 = Power up, 1 = Initialising, 2 = Ready for command, 4 = Wait response
static uint32_t simStartupDeadline = 0;
static uint32_t nextGPSEnableRetryAt = 0;
static uint32_t gpsEnableAcceptedAt = 0;
static char simBuffer[1024];
static size_t simBufferLength = 0;
static SIM7600Commands pendingCommand = GPS;
static bool simCommandPending = false;
static uint32_t simCommandSentAt = 0;
static uint16_t queuedSimCommands = 0;
static uint8_t activeSimCommandType = 0;
static bool activeGPSEnableState = false;
static uint8_t cellularDataState = 0;
static uint32_t nextCellularDataAttemptAt = 0;
static bool cellularTestBypassGpsWait = false;
static bool internetProbeComplete = false;
static bool internetProbeSucceeded = false;
static char cellularDataFailureMessage[96] = {0};
static char lastCellularModemResponse[160] = {0};
static uint8_t mqttClientState = 0;
static bool mqttConnected = false;
static uint32_t nextMqttPublishAt = 0;
static char mqttFailureMessage[96] = {0};
static char lastMqttModemResponse[160] = {0};
static char mqttTopicBuffer[192] = {0};
static char mqttPayloadBuffer[160] = {0};
static uint32_t mqttPublishAttemptCount = 0;
static uint32_t mqttPublishSuccessCount = 0;
static uint32_t mqttConsecutiveFailureCount = 0;
static uint32_t lastMqttPublishAttemptAt = 0;
static uint32_t lastMqttPublishSuccessAt = 0;
static uint32_t telemetryHealthMonitoringSince = 0;
static uint32_t lastMqttSubscribeAttemptAt = 0;
static uint32_t lastMqttSubscribeSuccessAt = 0;
static char lastMqttPublishPayload[160] = {0};
static char lastMqttPublishAttribute[32] = {0};
static char lastMqttSubscribeTopic[128] = {0};
static char lastMqttConnectAuthMode[12] = "unknown";
static char lastMqttConnectClientID[48] = {0};
static bool mqttSubscribed = false;
static uint32_t lastLocationPublishSuccessAt = 0;
static char lastLocationPublishPayload[160] = {0};
static bool telemetryBatchActive = false;
static uint8_t telemetryBatchCursor = 0;
static uint32_t telemetryBatchStartedAt = 0;
static int8_t activeTelemetryAttributeIndex = -1;
static int8_t telemetryRetryAttributeIndex = -1;
static bool telemetryRetryUsed = false;
static bool openRemoteProvisioningActive = false;
static bool openRemoteProvisioningComplete = false;
static bool openRemoteProvisioningSuccess = false;
static bool openRemoteProvisioningRequestPublished = false;
static char openRemoteProvisioningRequest[OPENREMOTE_PROVISIONING_MAX_REQUEST_BYTES + 1] = {0};
static char openRemoteProvisioningStatus[160] = {0};
static char openRemoteProvisioningRealm[32] = {0};
static char openRemoteProvisioningAssetID[64] = {0};
static char openRemoteProvisioningResponse[384] = {0};
static size_t openRemoteProvisioningResponseLength = 0;
static uint8_t openRemoteProvisioningPayloadHeaderMatch = 0;
static bool openRemoteProvisioningWaitingForPayloadStart = false;
static bool openRemoteProvisioningPayloadCaptureActive = false;

static void QueueSimCommand(SIM7600Commands command);
static void ClearQueuedSimCommand(SIM7600Commands command);
static bool IsMqttCommand(SIM7600Commands command);
static bool IsSimResponseComplete(const char *response);
static void ResetSimResponseBuffer();

enum CellularDataState
{
    CELLULAR_DATA_IDLE = 0,
    CELLULAR_DATA_SET_APN,
    CELLULAR_DATA_ATTACH,
    CELLULAR_DATA_ACTIVATE_PDP,
    CELLULAR_DATA_CHECK_ADDRESS,
    CELLULAR_DATA_OPEN_NETWORK,
    CELLULAR_DATA_SET_DNS,
    CELLULAR_DATA_PROBE_INTERNET,
    CELLULAR_DATA_CONNECTED,
    CELLULAR_DATA_FAILED,
    CELLULAR_DATA_DISCONNECT
};

enum MqttClientState
{
    MQTT_CLIENT_IDLE = 0,
    MQTT_CLIENT_CLEAN_DISCONNECT,
    MQTT_CLIENT_CLEAN_RELEASE,
    MQTT_CLIENT_CLEAN_STOP,
    MQTT_CLIENT_START,
    MQTT_CLIENT_ACQUIRE,
    MQTT_CLIENT_SSL_VERSION,
    MQTT_CLIENT_SSL_AUTHMODE,
    MQTT_CLIENT_SSL_BIND,
    MQTT_CLIENT_CONNECT,
    MQTT_CLIENT_CONNECTED,
    MQTT_CLIENT_SET_TOPIC,
    MQTT_CLIENT_WRITE_TOPIC,
    MQTT_CLIENT_SET_PAYLOAD,
    MQTT_CLIENT_WRITE_PAYLOAD,
    MQTT_CLIENT_PUBLISH,
    MQTT_CLIENT_SET_SUBSCRIBE_TOPIC,
    MQTT_CLIENT_WRITE_SUBSCRIBE_TOPIC,
    MQTT_CLIENT_SET_UNSUBSCRIBE_TOPIC,
    MQTT_CLIENT_WRITE_UNSUBSCRIBE_TOPIC,
    MQTT_CLIENT_FAILED
};

float lat, lon, speed, alt, accuracy;
int vsat, usat, year, month, day, hour, minute, second;

bool GPSFix = false;
bool dataConnected = false;

CellularParameters CellularParams;
CellularConfigUnion CellularConfigData;

float simModuleTemp = 0.0f;

struct ParsedGPSFix
{
    float latitude;
    float longitude;
    float altitude;
    float speedKnots;
    int parsedDay;
    int parsedMonth;
    int parsedYear;
    int parsedHour;
    int parsedMinute;
    int parsedSecond;
};

struct TelemetryAttributeDescriptor
{
    uint32_t flag;
    const char *attributeName;
    uint8_t payloadKind;
    uint8_t sourceIndex;
};

enum TelemetryPayloadKind
{
    TELEMETRY_PAYLOAD_CONNECTED,
    TELEMETRY_PAYLOAD_IGNITION,
    TELEMETRY_PAYLOAD_WAKE_STATE,
    TELEMETRY_PAYLOAD_FW_VERSION,
    TELEMETRY_PAYLOAD_ERROR_STATUS,
    TELEMETRY_PAYLOAD_ANALOGUE_VALUE,
    TELEMETRY_PAYLOAD_CHANNEL_CURRENT,
    TELEMETRY_PAYLOAD_DIGITAL_VALUE,
    TELEMETRY_PAYLOAD_GPS_SPEED,
    TELEMETRY_PAYLOAD_IMU_ACCEL_X,
    TELEMETRY_PAYLOAD_IMU_ACCEL_Y,
    TELEMETRY_PAYLOAD_IMU_ACCEL_Z,
    TELEMETRY_PAYLOAD_IMU_GYRO_X,
    TELEMETRY_PAYLOAD_IMU_GYRO_Y,
    TELEMETRY_PAYLOAD_IMU_GYRO_Z,
    TELEMETRY_PAYLOAD_IMU_MAG_X,
    TELEMETRY_PAYLOAD_IMU_MAG_Y,
    TELEMETRY_PAYLOAD_IMU_MAG_Z,
    TELEMETRY_PAYLOAD_LOCATION,
    TELEMETRY_PAYLOAD_GPS_LATITUDE,
    TELEMETRY_PAYLOAD_GPS_LONGITUDE,
    TELEMETRY_PAYLOAD_SYSTEM_CURRENT,
    TELEMETRY_PAYLOAD_SYSTEM_TEMPERATURE,
    TELEMETRY_PAYLOAD_SYSTEM_VOLTAGE,
    TELEMETRY_PAYLOAD_TEMP_WARNING,
    TELEMETRY_PAYLOAD_UPTIME,
};

static const TelemetryAttributeDescriptor TELEMETRY_ATTRIBUTES[] = {
    {0, "Connected", TELEMETRY_PAYLOAD_CONNECTED, 0},
    {0, "Ignition", TELEMETRY_PAYLOAD_IGNITION, 0},
    {0, "WakeState", TELEMETRY_PAYLOAD_WAKE_STATE, 0},
    {TELEMETRY_UPLOAD_ANALOGUE1_VALUE, "Analogue1Value", TELEMETRY_PAYLOAD_ANALOGUE_VALUE, 0},
    {TELEMETRY_UPLOAD_ANALOGUE2_VALUE, "Analogue2Value", TELEMETRY_PAYLOAD_ANALOGUE_VALUE, 1},
    {TELEMETRY_UPLOAD_ANALOGUE3_VALUE, "Analogue3Value", TELEMETRY_PAYLOAD_ANALOGUE_VALUE, 2},
    {TELEMETRY_UPLOAD_ANALOGUE4_VALUE, "Analogue4Value", TELEMETRY_PAYLOAD_ANALOGUE_VALUE, 3},
    {TELEMETRY_UPLOAD_ANALOGUE5_VALUE, "Analogue5Value", TELEMETRY_PAYLOAD_ANALOGUE_VALUE, 4},
    {TELEMETRY_UPLOAD_ANALOGUE6_VALUE, "Analogue6Value", TELEMETRY_PAYLOAD_ANALOGUE_VALUE, 5},
    {TELEMETRY_UPLOAD_ANALOGUE7_VALUE, "Analogue7Value", TELEMETRY_PAYLOAD_ANALOGUE_VALUE, 6},
    {TELEMETRY_UPLOAD_ANALOGUE8_VALUE, "Analogue8Value", TELEMETRY_PAYLOAD_ANALOGUE_VALUE, 7},
    {TELEMETRY_UPLOAD_SYSTEM_CURRENT, "SystemCurrent", TELEMETRY_PAYLOAD_SYSTEM_CURRENT, 0},
    {TELEMETRY_UPLOAD_SYSTEM_TEMPERATURE, "SystemTemperature", TELEMETRY_PAYLOAD_SYSTEM_TEMPERATURE, 0},
    {TELEMETRY_UPLOAD_SYSTEM_TEMPERATURE, "TempWarning", TELEMETRY_PAYLOAD_TEMP_WARNING, 0},
    {TELEMETRY_UPLOAD_SYSTEM_VOLTAGE, "SystemVoltage", TELEMETRY_PAYLOAD_SYSTEM_VOLTAGE, 0},
    {TELEMETRY_UPLOAD_UPTIME, "Uptime", TELEMETRY_PAYLOAD_UPTIME, 0},
    {TELEMETRY_UPLOAD_GPS_SPEED, "GPSSpeed", TELEMETRY_PAYLOAD_GPS_SPEED, 0},
    {TELEMETRY_UPLOAD_LOCATION, "location", TELEMETRY_PAYLOAD_LOCATION, 0},
    {TELEMETRY_UPLOAD_LOCATION, "GPSLatitude", TELEMETRY_PAYLOAD_GPS_LATITUDE, 0},
    {TELEMETRY_UPLOAD_LOCATION, "GPSLongitude", TELEMETRY_PAYLOAD_GPS_LONGITUDE, 0},
    {TELEMETRY_UPLOAD_CHANNEL_CURRENTS, "Channel1Current", TELEMETRY_PAYLOAD_CHANNEL_CURRENT, 0},
    {TELEMETRY_UPLOAD_CHANNEL_CURRENTS, "Channel2Current", TELEMETRY_PAYLOAD_CHANNEL_CURRENT, 1},
    {TELEMETRY_UPLOAD_CHANNEL_CURRENTS, "Channel3Current", TELEMETRY_PAYLOAD_CHANNEL_CURRENT, 2},
    {TELEMETRY_UPLOAD_CHANNEL_CURRENTS, "Channel4Current", TELEMETRY_PAYLOAD_CHANNEL_CURRENT, 3},
    {TELEMETRY_UPLOAD_CHANNEL_CURRENTS, "Channel5Current", TELEMETRY_PAYLOAD_CHANNEL_CURRENT, 4},
    {TELEMETRY_UPLOAD_CHANNEL_CURRENTS, "Channel6Current", TELEMETRY_PAYLOAD_CHANNEL_CURRENT, 5},
    {TELEMETRY_UPLOAD_CHANNEL_CURRENTS, "Channel7Current", TELEMETRY_PAYLOAD_CHANNEL_CURRENT, 6},
    {TELEMETRY_UPLOAD_CHANNEL_CURRENTS, "Channel8Current", TELEMETRY_PAYLOAD_CHANNEL_CURRENT, 7},
    {TELEMETRY_UPLOAD_CHANNEL_CURRENTS, "Channel9Current", TELEMETRY_PAYLOAD_CHANNEL_CURRENT, 8},
    {TELEMETRY_UPLOAD_CHANNEL_CURRENTS, "Channel10Current", TELEMETRY_PAYLOAD_CHANNEL_CURRENT, 9},
    {TELEMETRY_UPLOAD_CHANNEL_CURRENTS, "Channel11Current", TELEMETRY_PAYLOAD_CHANNEL_CURRENT, 10},
    {TELEMETRY_UPLOAD_CHANNEL_CURRENTS, "Channel12Current", TELEMETRY_PAYLOAD_CHANNEL_CURRENT, 11},
    {TELEMETRY_UPLOAD_CHANNEL_CURRENTS, "Channel13Current", TELEMETRY_PAYLOAD_CHANNEL_CURRENT, 12},
    {TELEMETRY_UPLOAD_CHANNEL_CURRENTS, "Channel14Current", TELEMETRY_PAYLOAD_CHANNEL_CURRENT, 13},
    {TELEMETRY_UPLOAD_DIGITAL1_VALUE, "Digital1Value", TELEMETRY_PAYLOAD_DIGITAL_VALUE, 0},
    {TELEMETRY_UPLOAD_DIGITAL2_VALUE, "Digital2Value", TELEMETRY_PAYLOAD_DIGITAL_VALUE, 1},
    {TELEMETRY_UPLOAD_DIGITAL3_VALUE, "Digital3Value", TELEMETRY_PAYLOAD_DIGITAL_VALUE, 2},
    {TELEMETRY_UPLOAD_DIGITAL4_VALUE, "Digital4Value", TELEMETRY_PAYLOAD_DIGITAL_VALUE, 3},
    {TELEMETRY_UPLOAD_DIGITAL5_VALUE, "Digital5Value", TELEMETRY_PAYLOAD_DIGITAL_VALUE, 4},
    {TELEMETRY_UPLOAD_DIGITAL6_VALUE, "Digital6Value", TELEMETRY_PAYLOAD_DIGITAL_VALUE, 5},
    {TELEMETRY_UPLOAD_DIGITAL7_VALUE, "Digital7Value", TELEMETRY_PAYLOAD_DIGITAL_VALUE, 6},
    {TELEMETRY_UPLOAD_DIGITAL8_VALUE, "Digital8Value", TELEMETRY_PAYLOAD_DIGITAL_VALUE, 7},
    {TELEMETRY_UPLOAD_IMU_DATA, "IMUAccelX", TELEMETRY_PAYLOAD_IMU_ACCEL_X, 0},
    {TELEMETRY_UPLOAD_IMU_DATA, "IMUAccelY", TELEMETRY_PAYLOAD_IMU_ACCEL_Y, 0},
    {TELEMETRY_UPLOAD_IMU_DATA, "IMUAccelZ", TELEMETRY_PAYLOAD_IMU_ACCEL_Z, 0},
    {TELEMETRY_UPLOAD_IMU_DATA, "IMUGyroX", TELEMETRY_PAYLOAD_IMU_GYRO_X, 0},
    {TELEMETRY_UPLOAD_IMU_DATA, "IMUGyroY", TELEMETRY_PAYLOAD_IMU_GYRO_Y, 0},
    {TELEMETRY_UPLOAD_IMU_DATA, "IMUGyroZ", TELEMETRY_PAYLOAD_IMU_GYRO_Z, 0},
    {TELEMETRY_UPLOAD_IMU_DATA, "IMUMagX", TELEMETRY_PAYLOAD_IMU_MAG_X, 0},
    {TELEMETRY_UPLOAD_IMU_DATA, "IMUMagY", TELEMETRY_PAYLOAD_IMU_MAG_Y, 0},
    {TELEMETRY_UPLOAD_IMU_DATA, "IMUMagZ", TELEMETRY_PAYLOAD_IMU_MAG_Z, 0},
};

static constexpr uint8_t TELEMETRY_ATTRIBUTE_COUNT = sizeof(TELEMETRY_ATTRIBUTES) / sizeof(TELEMETRY_ATTRIBUTES[0]);

static int SplitCsvPreserveEmpty(char *buffer, char *fields[], int maxFields)
{
    int count = 0;
    char *fieldStart = buffer;

    while (count < maxFields)
    {
        fields[count++] = fieldStart;

        char *separator = strchr(fieldStart, ',');
        if (separator == nullptr)
        {
            break;
        }

        *separator = '\0';
        fieldStart = separator + 1;
    }

    return count;
}

static void ClearLiveGPSData()
{
    lat = 0.0f;
    lon = 0.0f;
    speed = 0.0f;
    alt = 0.0f;
    accuracy = 0.0f;
    vsat = 0;
    usat = 0;
}

static void UpdateGPSFixState(bool hasCurrentFix, uint32_t now)
{
    if (hasCurrentFix)
    {
        GPSFix = true;
        return;
    }

    if (hasAcceptedGPSFix && lastGPSTime != 0 && (now - lastGPSTime) <= GPS_FIX_GRACE_PERIOD_MS)
    {
        GPSFix = true;
        return;
    }

    if (lastGNSSFixSeenAt != 0 && (now - lastGNSSFixSeenAt) <= GPS_FIX_GRACE_PERIOD_MS)
    {
        GPSFix = true;
        return;
    }

    GPSFix = false;
    ClearLiveGPSData();
}

static float KnotsToMetresPerSecond(float knots)
{
    return knots * 0.514444f;
}

static float DegreesToRadians(float degrees)
{
    return degrees * 0.01745329252f;
}

static float GreatCircleDistanceMetres(float startLat, float startLon, float endLat, float endLon)
{
    float startLatRad = DegreesToRadians(startLat);
    float endLatRad = DegreesToRadians(endLat);
    float deltaLat = DegreesToRadians(endLat - startLat);
    float deltaLon = DegreesToRadians(endLon - startLon);

    float sinLat = sinf(deltaLat * 0.5f);
    float sinLon = sinf(deltaLon * 0.5f);
    float a = (sinLat * sinLat) + (cosf(startLatRad) * cosf(endLatRad) * sinLon * sinLon);
    float clampedA = fminf(1.0f, fmaxf(0.0f, a));
    float c = 2.0f * atan2f(sqrtf(clampedA), sqrtf(1.0f - clampedA));
    return EARTH_RADIUS_METRES * c;
}

static int32_t DaysFromCivilDate(int32_t fullYear, uint32_t month, uint32_t day)
{
    fullYear -= month <= 2U;
    const int32_t era = (fullYear >= 0 ? fullYear : fullYear - 399) / 400;
    const uint32_t yearOfEra = (uint32_t)(fullYear - era * 400);
    const uint32_t monthPrime = month + (month > 2U ? (uint32_t)-3 : 9U);
    const uint32_t dayOfYear = (153U * monthPrime + 2U) / 5U + day - 1U;
    const uint32_t dayOfEra = yearOfEra * 365U + yearOfEra / 4U - yearOfEra / 100U + dayOfYear;
    return era * 146097 + (int32_t)dayOfEra - 719468;
}

static uint32_t BuildAcceptedGpsEpochSeconds()
{
    if (year < 2024 || year > 2099 || month < 1 || month > 12 || day < 1 || day > 31 || hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 59)
    {
        return 0;
    }

    int32_t days = DaysFromCivilDate(year, (uint32_t)month, (uint32_t)day);
    int64_t epochSeconds = ((int64_t)days * 86400LL) + ((int64_t)hour * 3600LL) + ((int64_t)minute * 60LL) + (int64_t)second;
    return epochSeconds > 0 && epochSeconds <= UINT32_MAX ? (uint32_t)epochSeconds : 0;
}

static bool IsCandidateFixPlausible(const ParsedGPSFix &candidate, uint32_t now)
{
    if (!isfinite(candidate.latitude) || !isfinite(candidate.longitude) || !isfinite(candidate.altitude) || !isfinite(candidate.speedKnots))
    {
        return false;
    }

    if (candidate.latitude < -90.0f || candidate.latitude > 90.0f || candidate.longitude < -180.0f || candidate.longitude > 180.0f || candidate.speedKnots < 0.0f)
    {
        return false;
    }

    float candidateSpeedMps = KnotsToMetresPerSecond(candidate.speedKnots);
    if (candidateSpeedMps > GPS_FILTER_MAX_SPEED_MPS)
    {
        return false;
    }

    uint32_t elapsedMs = now - lastGPSTime;
    if (!hasAcceptedGPSFix || lastGPSTime == 0 || elapsedMs == 0 || elapsedMs > GPS_FILTER_RESET_INTERVAL_MS)
    {
        return true;
    }

    float elapsedSeconds = elapsedMs / 1000.0f;
    float previousSpeedMps = KnotsToMetresPerSecond(lastAcceptedSpeedKnots);
    float maxAllowedSpeedMps = previousSpeedMps + (GPS_FILTER_MAX_ACCEL_MPS2 * elapsedSeconds) + GPS_FILTER_SPEED_MARGIN_MPS;
    if (candidateSpeedMps > maxAllowedSpeedMps)
    {
        return false;
    }

    float travelledDistanceMetres = GreatCircleDistanceMetres(lastAcceptedLat, lastAcceptedLon, candidate.latitude, candidate.longitude);
    float impliedSpeedMps = travelledDistanceMetres / elapsedSeconds;
    float maxAllowedTravelSpeedMps = fminf(
        GPS_FILTER_MAX_SPEED_MPS,
        fmaxf(previousSpeedMps, candidateSpeedMps) + (GPS_FILTER_MAX_ACCEL_MPS2 * elapsedSeconds) + GPS_FILTER_SPEED_MARGIN_MPS);

    return impliedSpeedMps <= maxAllowedTravelSpeedMps;
}

static void AcceptCandidateFix(const ParsedGPSFix &candidate, uint32_t now)
{
    lat = candidate.latitude;
    lon = candidate.longitude;
    alt = candidate.altitude;
    speed = candidate.speedKnots;
    day = candidate.parsedDay;
    month = candidate.parsedMonth;
    year = candidate.parsedYear;
    hour = candidate.parsedHour;
    minute = candidate.parsedMinute;
    second = candidate.parsedSecond;

    lastAcceptedLat = candidate.latitude;
    lastAcceptedLon = candidate.longitude;
    lastAcceptedSpeedKnots = candidate.speedKnots;
    lastGPSTime = now;
    hasAcceptedGPSFix = true;
    GPSFix = true;
}

static void ResetAcceptedGPSFixState()
{
    hasAcceptedGPSFix = false;
    hasSeenGNSSResponse = false;
    lastGNSSFixSeenAt = 0;
    lastGNSSInfoResponse[0] = '\0';
    snprintf(lastGPSParseStatus, sizeof(lastGPSParseStatus), "No GNSS response parsed.");
    lastAcceptedLat = 0.0f;
    lastAcceptedLon = 0.0f;
    lastAcceptedSpeedKnots = 0.0f;
    lastGPSTime = 0;
    ClearLiveGPSData();
}

static bool CanDispatchNonGPSCommandNow(uint32_t now)
{
    if (!SystemParams.AllowGPS)
    {
        return true;
    }

    return (int32_t)(GPSTimer - now) > (int32_t)SIM_NON_GPS_GUARD_TIME_MS;
}

static bool IsCellularDataEnabled()
{
    return SystemParams.AllowData != 0 &&
           CellularParams.APN[0] != '\0';
}

static bool CanStartCellularDataConnection()
{
    return true;
}

static uint32_t GetActiveSimCommandTimeoutMs()
{
    if (activeSimCommandType == SIM_ACTIVE_COMMAND_QUEUED &&
        pendingCommand == MQTT_CONNECT &&
        mqttClientState == MQTT_CLIENT_CONNECT)
    {
        return SIM7600_MQTT_CONNECT_RESPONSE_TIMEOUT_MS;
    }

    if (activeSimCommandType == SIM_ACTIVE_COMMAND_QUEUED &&
        pendingCommand == MQTT_PUBLISH &&
        (mqttClientState == MQTT_CLIENT_SET_TOPIC || mqttClientState == MQTT_CLIENT_SET_PAYLOAD ||
         mqttClientState == MQTT_CLIENT_WRITE_TOPIC || mqttClientState == MQTT_CLIENT_WRITE_PAYLOAD))
    {
        return SIM7600_MQTT_PROMPT_TIMEOUT_MS;
    }

    if (activeSimCommandType == SIM_ACTIVE_COMMAND_QUEUED &&
        (pendingCommand == MQTT_SUBSCRIBE || pendingCommand == MQTT_UNSUBSCRIBE) &&
        (mqttClientState == MQTT_CLIENT_SET_SUBSCRIBE_TOPIC || mqttClientState == MQTT_CLIENT_SET_UNSUBSCRIBE_TOPIC))
    {
        return SIM7600_MQTT_PROMPT_TIMEOUT_MS;
    }

    if (activeSimCommandType == SIM_ACTIVE_COMMAND_QUEUED &&
        pendingCommand == MQTT_PUBLISH &&
        mqttClientState == MQTT_CLIENT_PUBLISH)
    {
        return SIM7600_MQTT_PUBLISH_RESPONSE_TIMEOUT_MS;
    }

    if (activeSimCommandType == SIM_ACTIVE_COMMAND_QUEUED &&
        IsMqttCommand(pendingCommand))
    {
        return SIM7600_DATA_RESPONSE_TIMEOUT_MS;
    }

    if (activeSimCommandType == SIM_ACTIVE_COMMAND_QUEUED &&
        pendingCommand == CELLULAR_CONNECT &&
        (cellularDataState == CELLULAR_DATA_OPEN_NETWORK || cellularDataState == CELLULAR_DATA_PROBE_INTERNET))
    {
        return SIM7600_DNS_RESPONSE_TIMEOUT_MS;
    }

    if (activeSimCommandType == SIM_ACTIVE_COMMAND_QUEUED &&
        (pendingCommand == CELLULAR_CONNECT || pendingCommand == CELLULAR_DISCONNECT))
    {
        return SIM7600_DATA_RESPONSE_TIMEOUT_MS;
    }

    return SIM7600_RESPONSE_TIMEOUT_MS;
}

static void ScheduleCellularDataRetry(uint32_t now)
{
    dataConnected = false;
    internetProbeComplete = false;
    internetProbeSucceeded = false;
    cellularDataState = CELLULAR_DATA_IDLE;
    nextCellularDataAttemptAt = now + SIM7600_DATA_RETRY_MS;
}

static void FailCellularDataTest(const char *stage, bool responseError, bool commandTimedOut)
{
    dataConnected = false;
    cellularTestBypassGpsWait = false;
    internetProbeComplete = false;
    internetProbeSucceeded = false;
    cellularDataState = CELLULAR_DATA_FAILED;
    snprintf(cellularDataFailureMessage,
             sizeof(cellularDataFailureMessage),
             "%s %s.",
             stage,
             commandTimedOut ? "timed out" : (responseError ? "returned ERROR" : "did not return OK"));
}

static void StartCellularDataConnection(uint32_t now)
{
    dataConnected = false;
    internetProbeComplete = false;
    internetProbeSucceeded = false;
    cellularDataFailureMessage[0] = '\0';
    cellularDataState = CELLULAR_DATA_SET_APN;
    nextCellularDataAttemptAt = now + SIM7600_DATA_RETRY_MS;
    QueueSimCommand(CELLULAR_CONNECT);
}

static bool IsCellularInternetReady()
{
    return dataConnected &&
           cellularDataState == CELLULAR_DATA_CONNECTED &&
           internetProbeComplete &&
           internetProbeSucceeded;
}

static const char *GetInternetProbeHost()
{
    return CellularParams.OpenRemoteHost[0] != '\0' ? CellularParams.OpenRemoteHost : INTERNET_PROBE_HOST;
}

static const char *GetCellularDataStateName()
{
    switch (cellularDataState)
    {
    case CELLULAR_DATA_IDLE:
        return "Idle";
    case CELLULAR_DATA_SET_APN:
        return "SetAPN";
    case CELLULAR_DATA_ATTACH:
        return "Attach";
    case CELLULAR_DATA_ACTIVATE_PDP:
        return "ActivatePDP";
    case CELLULAR_DATA_CHECK_ADDRESS:
        return "CheckAddress";
    case CELLULAR_DATA_OPEN_NETWORK:
        return "OpenNetwork";
    case CELLULAR_DATA_SET_DNS:
        return "SetDNS";
    case CELLULAR_DATA_PROBE_INTERNET:
        return "ProbeInternet";
    case CELLULAR_DATA_CONNECTED:
        return "Connected";
    case CELLULAR_DATA_FAILED:
        return "Failed";
    case CELLULAR_DATA_DISCONNECT:
        return "Disconnect";
    default:
        return "Unknown";
    }
}

static const char *GetSimStateName()
{
    switch (SIM7600State)
    {
    case 0:
        return "PowerUp";
    case 1:
        return "Initialising";
    case 2:
        return "Ready";
    default:
        return "Unknown";
    }
}

static const char *GetSimCommandName(SIM7600Commands command)
{
    switch (command)
    {
    case GPS:
        return "GPS";
    case SIGNAL_QUALITY:
        return "SignalQuality";
    case NETWORK_MODE:
        return "NetworkMode";
    case MODULE_TEMPERATURE:
        return "ModuleTemperature";
    case CELLULAR_CONNECT:
        return "CellularConnect";
    case CELLULAR_DISCONNECT:
        return "CellularDisconnect";
    case MQTT_CONNECT:
        return "MqttConnect";
    case MQTT_PUBLISH:
        return "MqttPublish";
    case MQTT_SUBSCRIBE:
        return "MqttSubscribe";
    case MQTT_UNSUBSCRIBE:
        return "MqttUnsubscribe";
    default:
        return "Other";
    }
}

static uint16_t CommandBit(SIM7600Commands command)
{
    return static_cast<uint16_t>(1U << static_cast<uint8_t>(command));
}

static bool IsCellularDataCommand(SIM7600Commands command)
{
    return command == CELLULAR_CONNECT || command == CELLULAR_DISCONNECT;
}

static bool IsMqttCommand(SIM7600Commands command)
{
    return command == MQTT_CONNECT || command == MQTT_PUBLISH || command == MQTT_SUBSCRIBE || command == MQTT_UNSUBSCRIBE || command == MQTT_DISCONNECT || command == MQTT_PING || command == MQTT_STATUS;
}

static bool IsMqttConfigured()
{
    if (openRemoteProvisioningActive)
    {
        return CellularParams.OpenRemoteHost[0] != '\0' &&
               CellularParams.UseTLS != 0 &&
               CellularParams.ClientID[0] != '\0' &&
               openRemoteProvisioningRequest[0] != '\0';
    }

    return CellularParams.OpenRemoteHost[0] != '\0' &&
           CellularParams.UseTLS != 0 &&
           CellularParams.ClientID[0] != '\0' &&
           CellularParams.MQTTUsername[0] != '\0' &&
           CellularParams.MQTTPassword[0] != '\0' &&
           CellularParams.PublishTopic[0] != '\0';
}
static bool IsOpenRemotePublishTopic()
{
    return strstr(CellularParams.PublishTopic, "/writeattributevalue/") != nullptr;
}
static bool IsOpenRemoteMqttUsernameRealmPrefixed()
{
    return strchr(CellularParams.MQTTUsername, ':') != nullptr;
}

static bool IsOpenRemotePublishTopicClientIDMatched()
{
    if (!IsOpenRemotePublishTopic())
    {
        return true;
    }

    const char *firstSeparator = strchr(CellularParams.PublishTopic, '/');
    const char *secondSeparator = firstSeparator != nullptr ? strchr(firstSeparator + 1, '/') : nullptr;
    if (firstSeparator == nullptr || secondSeparator == nullptr)
    {
        return false;
    }

    size_t topicClientIDLength = static_cast<size_t>(secondSeparator - firstSeparator - 1);
    return strlen(CellularParams.ClientID) == topicClientIDLength &&
           strncmp(firstSeparator + 1, CellularParams.ClientID, topicClientIDLength) == 0;
}

static bool BuildOpenRemoteProvisioningResponseTopic()
{
    int written = snprintf(mqttTopicBuffer,
                           sizeof(mqttTopicBuffer),
                           "provisioning/%s/response",
                           CellularParams.ClientID);
    return written > 0 && static_cast<size_t>(written) < sizeof(mqttTopicBuffer);
}

static bool ExtractJsonStringValue(const char *json, const char *key, char *destination, size_t destinationSize)
{
    if (json == nullptr || key == nullptr || destination == nullptr || destinationSize == 0)
    {
        return false;
    }

    destination[0] = '\0';
    const char *keyPosition = strstr(json, key);
    if (keyPosition == nullptr)
    {
        return false;
    }

    const char *colon = strchr(keyPosition + strlen(key), ':');
    if (colon == nullptr)
    {
        return false;
    }

    const char *valueStart = strchr(colon, '"');
    if (valueStart == nullptr)
    {
        return false;
    }
    valueStart++;

    const char *valueEnd = strchr(valueStart, '"');
    if (valueEnd == nullptr || valueEnd == valueStart)
    {
        return false;
    }

    size_t valueLength = static_cast<size_t>(valueEnd - valueStart);
    if (valueLength >= destinationSize)
    {
        valueLength = destinationSize - 1;
    }

    memcpy(destination, valueStart, valueLength);
    destination[valueLength] = '\0';
    return true;
}

static bool ExtractOpenRemoteAssetID(const char *json, char *destination, size_t destinationSize)
{
    if (ExtractJsonStringValue(json, "\"assetId\"", destination, destinationSize) ||
        ExtractJsonStringValue(json, "\"assetID\"", destination, destinationSize) ||
        ExtractJsonStringValue(json, "\"id\"", destination, destinationSize))
    {
        return true;
    }

    const char *assetObject = json != nullptr ? strstr(json, "\"asset\"") : nullptr;
    if (assetObject == nullptr)
    {
        return false;
    }

    return ExtractJsonStringValue(assetObject, "\"id\"", destination, destinationSize);
}

static void SanitiseOpenRemoteProvisioningResponseForStatus()
{
    for (size_t i = 0; openRemoteProvisioningResponse[i] != '\0'; i++)
    {
        if (openRemoteProvisioningResponse[i] == '\r' || openRemoteProvisioningResponse[i] == '\n' || openRemoteProvisioningResponse[i] == '\t')
        {
            openRemoteProvisioningResponse[i] = ' ';
        }
    }
}

static void ResetOpenRemoteProvisioningReceiveCapture()
{
    openRemoteProvisioningResponse[0] = '\0';
    openRemoteProvisioningResponseLength = 0;
    openRemoteProvisioningPayloadHeaderMatch = 0;
    openRemoteProvisioningWaitingForPayloadStart = false;
    openRemoteProvisioningPayloadCaptureActive = false;
}

static void CaptureOpenRemoteProvisioningIncomingChar(char incoming)
{
    if (!openRemoteProvisioningActive || openRemoteProvisioningComplete)
    {
        return;
    }

    if (openRemoteProvisioningPayloadCaptureActive)
    {
        if (openRemoteProvisioningResponseLength < sizeof(openRemoteProvisioningResponse) - 1)
        {
            openRemoteProvisioningResponse[openRemoteProvisioningResponseLength++] = incoming;
            openRemoteProvisioningResponse[openRemoteProvisioningResponseLength] = '\0';
        }
        return;
    }

    if (openRemoteProvisioningWaitingForPayloadStart)
    {
        if (incoming == '\n')
        {
            openRemoteProvisioningWaitingForPayloadStart = false;
            openRemoteProvisioningPayloadCaptureActive = true;
        }
        return;
    }

    static const char payloadHeader[] = "+CMQTTRXPAYLOAD:";
    if (incoming == payloadHeader[openRemoteProvisioningPayloadHeaderMatch])
    {
        openRemoteProvisioningPayloadHeaderMatch++;
        if (openRemoteProvisioningPayloadHeaderMatch == sizeof(payloadHeader) - 1)
        {
            openRemoteProvisioningPayloadHeaderMatch = 0;
            openRemoteProvisioningWaitingForPayloadStart = true;
            openRemoteProvisioningResponse[0] = '\0';
            openRemoteProvisioningResponseLength = 0;
        }
    }
    else
    {
        openRemoteProvisioningPayloadHeaderMatch = incoming == payloadHeader[0] ? 1 : 0;
    }
}

static bool ExtractMqttReceivePayload(const char *response, char *destination, size_t destinationSize)
{
    if (response == nullptr || destination == nullptr || destinationSize == 0 || strstr(response, "+CMQTTRXEND:") == nullptr)
    {
        return false;
    }

    const char *payloadHeader = strstr(response, "+CMQTTRXPAYLOAD:");
    if (payloadHeader == nullptr)
    {
        return false;
    }

    int client = -1;
    int payloadLength = 0;
    if (sscanf(payloadHeader, "+CMQTTRXPAYLOAD: %d,%d", &client, &payloadLength) != 2 || client != 0 || payloadLength <= 0)
    {
        return false;
    }

    const char *payloadStart = strchr(payloadHeader, '\n');
    if (payloadStart == nullptr)
    {
        return false;
    }
    payloadStart++;
    while (*payloadStart == '\r' || *payloadStart == '\n')
    {
        payloadStart++;
    }

    size_t copyLength = static_cast<size_t>(payloadLength);
    if (copyLength >= destinationSize)
    {
        copyLength = destinationSize - 1;
    }

    memcpy(destination, payloadStart, copyLength);
    destination[copyLength] = '\0';
    return true;
}

static void CaptureOpenRemoteProvisioningResponseIfAvailable()
{
    if (!openRemoteProvisioningActive || openRemoteProvisioningComplete || simCommandPending)
    {
        return;
    }

    char responsePayload[sizeof(openRemoteProvisioningResponse)] = {0};
    if (strstr(simBuffer, "+CMQTTRXEND:") == nullptr)
    {
        return;
    }

    openRemoteProvisioningPayloadCaptureActive = false;
    if (openRemoteProvisioningResponse[0] == '\0' && !ExtractMqttReceivePayload(simBuffer, openRemoteProvisioningResponse, sizeof(openRemoteProvisioningResponse)))
    {
        return;
    }

    snprintf(responsePayload, sizeof(responsePayload), "%s", openRemoteProvisioningResponse);
    ExtractJsonStringValue(responsePayload, "\"realm\"", openRemoteProvisioningRealm, sizeof(openRemoteProvisioningRealm));
    ExtractOpenRemoteAssetID(responsePayload, openRemoteProvisioningAssetID, sizeof(openRemoteProvisioningAssetID));

    char responseType[24] = {0};
    ExtractJsonStringValue(responsePayload, "\"type\"", responseType, sizeof(responseType));
    bool responseAccepted = strcmp(responseType, "success") == 0 ||
                            strcmp(responseType, "ThingAsset") == 0 ||
                            openRemoteProvisioningAssetID[0] != '\0' ||
                            strstr(responsePayload, "\"success\"") != nullptr ||
                            strstr(responsePayload, "\"ClientId\"") != nullptr;

    if (responseAccepted && openRemoteProvisioningAssetID[0] != '\0')
    {
        snprintf(openRemoteProvisioningStatus, sizeof(openRemoteProvisioningStatus), "OpenRemote provisioning response received.");
        openRemoteProvisioningSuccess = true;
    }
    else if (responseAccepted)
    {
        snprintf(openRemoteProvisioningStatus, sizeof(openRemoteProvisioningStatus), "OpenRemote provisioning accepted; response did not include an asset ID.");
        openRemoteProvisioningSuccess = true;
    }
    else
    {
        snprintf(openRemoteProvisioningStatus, sizeof(openRemoteProvisioningStatus), "OpenRemote provisioning response did not report success.");
        openRemoteProvisioningSuccess = false;
    }

    SanitiseOpenRemoteProvisioningResponseForStatus();
    openRemoteProvisioningComplete = true;
    ResetSimResponseBuffer();
}

static const char *GetMqttConnectClientID()
{
    if (!openRemoteProvisioningActive)
    {
        return CellularParams.ClientID;
    }

    static char provisioningClientID[24] = {0};
    size_t clientIDLength = strlen(CellularParams.ClientID);
    const char *suffix = clientIDLength > 18 ? CellularParams.ClientID + clientIDLength - 18 : CellularParams.ClientID;
    snprintf(provisioningClientID, sizeof(provisioningClientID), "pdm-%s", suffix);
    return provisioningClientID;
}

static const char *GetMqttConnectUsername()
{
    if (!IsOpenRemotePublishTopic() || CellularParams.ClientID[0] == '\0')
    {
        return CellularParams.MQTTUsername;
    }

    const char *realmEnd = strchr(CellularParams.PublishTopic, '/');
    if (realmEnd == nullptr || realmEnd == CellularParams.PublishTopic)
    {
        return CellularParams.MQTTUsername;
    }

    static char openRemoteUsername[64] = {0};
    int written = snprintf(openRemoteUsername,
                           sizeof(openRemoteUsername),
                           "%.*s:ps-%s",
                           static_cast<int>(realmEnd - CellularParams.PublishTopic),
                           CellularParams.PublishTopic,
                           CellularParams.ClientID);
    return written > 0 && static_cast<size_t>(written) < sizeof(openRemoteUsername) ? openRemoteUsername : CellularParams.MQTTUsername;
}

static uint32_t GetConfiguredTelemetryUploadMask()
{
    return CellularParams.TelemetryUploadMask != 0 ? CellularParams.TelemetryUploadMask : TELEMETRY_UPLOAD_DEFAULT_MASK;
}

static bool IsTelemetryAttributeSelected(uint8_t attributeIndex, uint32_t uploadMask)
{
    if (attributeIndex >= TELEMETRY_ATTRIBUTE_COUNT)
    {
        return false;
    }

    return TELEMETRY_ATTRIBUTES[attributeIndex].flag == 0 || (uploadMask & TELEMETRY_ATTRIBUTES[attributeIndex].flag) != 0;
}

static uint8_t CountConfiguredTelemetryAttributes()
{
    uint32_t uploadMask = GetConfiguredTelemetryUploadMask();
    uint8_t count = 0;
    for (uint8_t attributeIndex = 0; attributeIndex < TELEMETRY_ATTRIBUTE_COUNT; attributeIndex++)
    {
        if (IsTelemetryAttributeSelected(attributeIndex, uploadMask))
        {
            count++;
        }
    }

    return count;
}

static uint8_t CountTelemetryAttributesBeforeCursor()
{
    if (!telemetryBatchActive)
    {
        return 0;
    }

    uint32_t uploadMask = GetConfiguredTelemetryUploadMask();
    uint8_t count = 0;
    uint8_t cursorLimit = telemetryBatchCursor < TELEMETRY_ATTRIBUTE_COUNT ? telemetryBatchCursor : TELEMETRY_ATTRIBUTE_COUNT;
    for (uint8_t attributeIndex = 0; attributeIndex < cursorLimit; attributeIndex++)
    {
        if (IsTelemetryAttributeSelected(attributeIndex, uploadMask))
        {
            count++;
        }
    }

    return count;
}

static uint32_t GetConfiguredMqttPublishIntervalMs()
{
    return CellularParams.PublishIntervalMs < CELLULAR_DEFAULT_PUBLISH_INTERVAL_MS ? CELLULAR_DEFAULT_PUBLISH_INTERVAL_MS : CellularParams.PublishIntervalMs;
}

static uint32_t GetMqttPublishIntervalMs()
{
    return GetConfiguredMqttPublishIntervalMs();
}

static uint32_t GetTelemetryOfflineTimeoutMs()
{
    static constexpr uint32_t MINIMUM_TELEMETRY_OFFLINE_TIMEOUT_MS = 120000UL;
    static constexpr uint32_t MISSED_TELEMETRY_INTERVAL_LIMIT = 10UL;
    uint32_t publishIntervalMs = GetMqttPublishIntervalMs();
    uint32_t intervalTimeoutMs = publishIntervalMs > UINT32_MAX / MISSED_TELEMETRY_INTERVAL_LIMIT
                                     ? UINT32_MAX
                                     : publishIntervalMs * MISSED_TELEMETRY_INTERVAL_LIMIT;
    return intervalTimeoutMs > MINIMUM_TELEMETRY_OFFLINE_TIMEOUT_MS ? intervalTimeoutMs : MINIMUM_TELEMETRY_OFFLINE_TIMEOUT_MS;
}

bool IsTelemetryOffline()
{
    bool telemetryConfigured = IsCellularDataEnabled() && IsMqttConfigured() && !openRemoteProvisioningActive;
    if (!telemetryConfigured)
    {
        telemetryHealthMonitoringSince = 0;
        return false;
    }

    uint32_t now = millis();
    if (telemetryHealthMonitoringSince == 0)
    {
        telemetryHealthMonitoringSince = now;
        return false;
    }

    bool hasConfirmedTelemetry = lastMqttPublishSuccessAt != 0 && strcmp(lastMqttPublishAttribute, "OpenRemoteProvisioning") != 0;
    uint32_t lastHealthyAt = hasConfirmedTelemetry ? lastMqttPublishSuccessAt : telemetryHealthMonitoringSince;
    return now - lastHealthyAt > GetTelemetryOfflineTimeoutMs();
}

static const char *GetMissingMqttSettingMessage()
{
    if (CellularParams.OpenRemoteHost[0] == '\0')
    {
        return "Enter MQTT host.";
    }

    if (CellularParams.UseTLS == 0)
    {
        return "Enable TLS for OpenRemote MQTT.";
    }

    if (CellularParams.ClientID[0] == '\0')
    {
        return "Enter MQTT client ID.";
    }

    if (CellularParams.MQTTUsername[0] == '\0')
    {
        return "Enter MQTT username.";
    }

    if (CellularParams.MQTTPassword[0] == '\0')
    {
        return "Enter MQTT password/secret.";
    }

    if (CellularParams.PublishTopic[0] == '\0')
    {
        return "Enter OpenRemote realm and asset ID so Cortex can prepare telemetry topics.";
    }

    return "MQTT settings are incomplete.";
}

static bool BuildMqttPublishTopic(const char *attributeName, bool publishAttributeEvent)
{
    if (CellularParams.PublishTopic[0] == '\0' || attributeName == nullptr || attributeName[0] == '\0')
    {
        return false;
    }

    const char *firstSeparator = strchr(CellularParams.PublishTopic, '/');
    const char *secondSeparator = firstSeparator != nullptr ? strchr(firstSeparator + 1, '/') : nullptr;
    const char *thirdSeparator = secondSeparator != nullptr ? strchr(secondSeparator + 1, '/') : nullptr;
    const char *fourthSeparator = thirdSeparator != nullptr ? strchr(thirdSeparator + 1, '/') : nullptr;
    if (thirdSeparator == nullptr || fourthSeparator == nullptr)
    {
        return false;
    }

    size_t prefixLength = static_cast<size_t>(secondSeparator + 1 - CellularParams.PublishTopic);
    int written = snprintf(mqttTopicBuffer,
                           sizeof(mqttTopicBuffer),
                           "%.*s%s/%s%s",
                           static_cast<int>(prefixLength),
                           CellularParams.PublishTopic,
                           publishAttributeEvent ? "writeattribute" : "writeattributevalue",
                           attributeName,
                           fourthSeparator);
    return written > 0 && (size_t)written < sizeof(mqttTopicBuffer);
}

static bool BuildTelemetryPayload(uint8_t attributeIndex)
{
    if (attributeIndex >= TELEMETRY_ATTRIBUTE_COUNT)
    {
        return false;
    }

    const TelemetryAttributeDescriptor &attribute = TELEMETRY_ATTRIBUTES[attributeIndex];
    int written = -1;
    switch (attribute.payloadKind)
    {
    case TELEMETRY_PAYLOAD_CONNECTED:
        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "true");
        break;
    case TELEMETRY_PAYLOAD_IGNITION:
        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%s", digitalRead(IGN_INPUT) ? "true" : "false");
        break;
    case TELEMETRY_PAYLOAD_WAKE_STATE:
        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "\"%s\"", WakeSource == WAKE_SOURCE_IMU ? "imu" : "ignition");
        break;
    case TELEMETRY_PAYLOAD_FW_VERSION:
        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "\"%s\"", FW_VER);
        break;
    case TELEMETRY_PAYLOAD_ERROR_STATUS:
        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%u", static_cast<unsigned int>(SystemRuntimeParams.ErrorFlags));
        break;
    case TELEMETRY_PAYLOAD_ANALOGUE_VALUE:
    {
        uint8_t inputIndex = attribute.sourceIndex;
        if (inputIndex >= NUM_ANA_CHANNELS)
        {
            return false;
        }

        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.3f", AnalogueIns[inputIndex].InputValue);
        break;
    }
    case TELEMETRY_PAYLOAD_CHANNEL_CURRENT:
    {
        uint8_t channelIndex = attribute.sourceIndex;
        if (channelIndex >= NUM_CHANNELS)
        {
            return false;
        }

        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.2f", ChannelRuntime[channelIndex].CurrentValue);
        break;
    }
    case TELEMETRY_PAYLOAD_DIGITAL_VALUE:
    {
        uint8_t inputIndex = attribute.sourceIndex;
        if (inputIndex >= NUM_DI_CHANNELS)
        {
            return false;
        }

        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%s", digitalRead(DIchannelInputPins[inputIndex]) ? "true" : "false");
        break;
    }
    case TELEMETRY_PAYLOAD_GPS_SPEED:
        if (!GPSFix)
        {
            return false;
        }
        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.2f", speed * 1.852f);
        break;
    case TELEMETRY_PAYLOAD_IMU_ACCEL_X:
    case TELEMETRY_PAYLOAD_IMU_ACCEL_Y:
    case TELEMETRY_PAYLOAD_IMU_ACCEL_Z:
    case TELEMETRY_PAYLOAD_IMU_GYRO_X:
    case TELEMETRY_PAYLOAD_IMU_GYRO_Y:
    case TELEMETRY_PAYLOAD_IMU_GYRO_Z:
    case TELEMETRY_PAYLOAD_IMU_MAG_X:
    case TELEMETRY_PAYLOAD_IMU_MAG_Y:
    case TELEMETRY_PAYLOAD_IMU_MAG_Z:
        if (!IMUOK)
        {
            return false;
        }
        switch (attribute.payloadKind)
        {
        case TELEMETRY_PAYLOAD_IMU_ACCEL_X:
            written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.3f", accelX);
            break;
        case TELEMETRY_PAYLOAD_IMU_ACCEL_Y:
            written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.3f", accelY);
            break;
        case TELEMETRY_PAYLOAD_IMU_ACCEL_Z:
            written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.3f", accelZ);
            break;
        case TELEMETRY_PAYLOAD_IMU_GYRO_X:
            written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.3f", gyroX);
            break;
        case TELEMETRY_PAYLOAD_IMU_GYRO_Y:
            written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.3f", gyroY);
            break;
        case TELEMETRY_PAYLOAD_IMU_GYRO_Z:
            written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.3f", gyroZ);
            break;
        case TELEMETRY_PAYLOAD_IMU_MAG_X:
            written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.3f", magX);
            break;
        case TELEMETRY_PAYLOAD_IMU_MAG_Y:
            written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.3f", magY);
            break;
        case TELEMETRY_PAYLOAD_IMU_MAG_Z:
            written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.3f", magZ);
            break;
        default:
            return false;
        }
        break;
    case TELEMETRY_PAYLOAD_LOCATION:
        if (!GPSFix)
        {
            return false;
        }
        written = snprintf(mqttPayloadBuffer,
                           sizeof(mqttPayloadBuffer),
                           "{\"value\":{\"type\":\"Point\",\"coordinates\":[%.6f,%.6f]},\"timestamp\":%lu000}",
                           lon,
                           lat,
                           (unsigned long)BuildAcceptedGpsEpochSeconds());
        break;
    case TELEMETRY_PAYLOAD_GPS_LATITUDE:
        if (!GPSFix)
        {
            return false;
        }
        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.6f", lat);
        break;
    case TELEMETRY_PAYLOAD_GPS_LONGITUDE:
        if (!GPSFix)
        {
            return false;
        }
        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.6f", lon);
        break;
    case TELEMETRY_PAYLOAD_SYSTEM_CURRENT:
        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.2f", SystemRuntimeParams.SystemCurrent);
        break;
    case TELEMETRY_PAYLOAD_SYSTEM_TEMPERATURE:
        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%ld", static_cast<long>(SystemRuntimeParams.SystemTemperature));
        break;
    case TELEMETRY_PAYLOAD_SYSTEM_VOLTAGE:
        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%.2f", SystemRuntimeParams.VBatt);
        break;
    case TELEMETRY_PAYLOAD_TEMP_WARNING:
        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%s", (SystemRuntimeParams.ErrorFlags & TEMP_WARNING) != 0 ? "true" : "false");
        break;
    case TELEMETRY_PAYLOAD_UPTIME:
        written = snprintf(mqttPayloadBuffer, sizeof(mqttPayloadBuffer), "%lu", static_cast<unsigned long>(millis() / 1000UL));
        break;
    default:
        return false;
    }

    return written > 0 && static_cast<size_t>(written) < sizeof(mqttPayloadBuffer);
}

static const char *GetActiveMqttPayload()
{
    return openRemoteProvisioningActive ? openRemoteProvisioningRequest : mqttPayloadBuffer;
}

static bool PrepareNextTelemetryPublish(uint32_t now)
{
    if (!telemetryBatchActive)
    {
        telemetryBatchActive = true;
        telemetryBatchCursor = 0;
        telemetryBatchStartedAt = now;
        telemetryRetryAttributeIndex = -1;
        telemetryRetryUsed = false;
    }

    uint32_t uploadMask = GetConfiguredTelemetryUploadMask();
    while (telemetryBatchCursor < TELEMETRY_ATTRIBUTE_COUNT)
    {
        uint8_t attributeIndex = telemetryBatchCursor++;
        if (!IsTelemetryAttributeSelected(attributeIndex, uploadMask))
        {
            continue;
        }

        if (!BuildTelemetryPayload(attributeIndex) || !BuildMqttPublishTopic(TELEMETRY_ATTRIBUTES[attributeIndex].attributeName, TELEMETRY_ATTRIBUTES[attributeIndex].payloadKind == TELEMETRY_PAYLOAD_LOCATION))
        {
            continue;
        }

        activeTelemetryAttributeIndex = static_cast<int8_t>(attributeIndex);
        mqttPublishAttemptCount++;
        lastMqttPublishAttemptAt = now;
        return true;
    }

    telemetryBatchActive = false;
    activeTelemetryAttributeIndex = -1;
    telemetryRetryAttributeIndex = -1;
    telemetryRetryUsed = false;
    uint32_t nextBatchAt = telemetryBatchStartedAt + GetMqttPublishIntervalMs();
    nextMqttPublishAt = (int32_t)(now - nextBatchAt) >= 0 ? now : nextBatchAt;
    telemetryBatchStartedAt = 0;
    return false;
}

static void RetryActiveTelemetryAttributeOnce()
{
    if (!telemetryBatchActive || activeTelemetryAttributeIndex < 0 || activeTelemetryAttributeIndex >= static_cast<int8_t>(TELEMETRY_ATTRIBUTE_COUNT))
    {
        return;
    }

    if (telemetryRetryAttributeIndex == activeTelemetryAttributeIndex && telemetryRetryUsed)
    {
        telemetryRetryAttributeIndex = -1;
        telemetryRetryUsed = false;
        return;
    }

    telemetryRetryAttributeIndex = activeTelemetryAttributeIndex;
    telemetryRetryUsed = true;
    telemetryBatchCursor = static_cast<uint8_t>(activeTelemetryAttributeIndex);
}

static void ClearTelemetryAttributeRetry(int8_t attributeIndex)
{
    if (telemetryRetryAttributeIndex == attributeIndex)
    {
        telemetryRetryAttributeIndex = -1;
        telemetryRetryUsed = false;
    }
}

static void CaptureLastMqttModemResponse()
{
    const char *resultStart = strstr(simBuffer, "+CMQTTCONNECT:");
    if (resultStart == nullptr)
    {
        resultStart = strstr(simBuffer, "+CMQTTPUB:");
    }
    if (resultStart == nullptr)
    {
        resultStart = strstr(simBuffer, "+CMQTTSUB:");
    }
    if (resultStart == nullptr)
    {
        resultStart = strstr(simBuffer, "+CMQTTUNSUB:");
    }
    if (resultStart == nullptr)
    {
        resultStart = strstr(simBuffer, "+CMQTTSTART:");
    }
    if (resultStart == nullptr)
    {
        resultStart = strstr(simBuffer, "+CMQTTDISC:");
    }
    if (resultStart == nullptr)
    {
        resultStart = strstr(simBuffer, "+CMQTTSTOP:");
    }

    if (resultStart == nullptr && strstr(simBuffer, "AT+CMQTTCONNECT=") != nullptr)
    {
        snprintf(lastMqttModemResponse, sizeof(lastMqttModemResponse), "AT+CMQTTCONNECT sent; waiting for modem result.");
        return;
    }

    const char *source = resultStart != nullptr ? resultStart : simBuffer;
    size_t writeIndex = 0;
    for (size_t readIndex = 0; source[readIndex] != '\0' && writeIndex < sizeof(lastMqttModemResponse) - 1; readIndex++)
    {
        char character = source[readIndex];
        if (character == '\r' || character == '\n')
        {
            character = ' ';
        }

        lastMqttModemResponse[writeIndex++] = character;
    }

    lastMqttModemResponse[writeIndex] = '\0';
}

static bool HasMqttPrompt(const char *response)
{
    return response != nullptr && strchr(response, '>') != nullptr;
}

static bool HasMqttToken(const char *response, const char *token)
{
    return response != nullptr && token != nullptr && strstr(response, token) != nullptr;
}

static bool WasMqttSingleCodeSuccessful(const char *response, const char *token)
{
    const char *result = response != nullptr ? strstr(response, token) : nullptr;
    if (result == nullptr)
    {
        return false;
    }

    result += strlen(token);
    while (*result == ' ' || *result == '\t')
    {
        result++;
    }

    int code = -1;
    return sscanf(result, "%d", &code) == 1 && code == 0;
}

static int GetMqttSingleResultCode(const char *response, const char *token)
{
    const char *result = response != nullptr ? strstr(response, token) : nullptr;
    if (result == nullptr)
    {
        return -1;
    }

    result += strlen(token);
    while (*result == ' ' || *result == '\t')
    {
        result++;
    }

    int code = -1;
    return sscanf(result, "%d", &code) == 1 ? code : -1;
}

static bool WasMqttClientCodeSuccessful(const char *response, const char *token)
{
    const char *result = response != nullptr ? strstr(response, token) : nullptr;
    if (result == nullptr)
    {
        return false;
    }

    result += strlen(token);
    while (*result == ' ' || *result == '\t')
    {
        result++;
    }

    int client = -1;
    int code = -1;
    return sscanf(result, "%d,%d", &client, &code) == 2 && client == 0 && code == 0;
}

static int GetMqttClientResultCode(const char *response, const char *token)
{
    const char *result = response != nullptr ? strstr(response, token) : nullptr;
    if (result == nullptr)
    {
        return -1;
    }

    result += strlen(token);
    while (*result == ' ' || *result == '\t')
    {
        result++;
    }

    int client = -1;
    int code = -1;
    if (sscanf(result, "%d,%d", &client, &code) != 2 || client != 0)
    {
        return -1;
    }

    return code;
}

static const char *GetMqttClientStateName()
{
    switch (mqttClientState)
    {
    case MQTT_CLIENT_IDLE:
        return "Idle";
    case MQTT_CLIENT_CLEAN_DISCONNECT:
        return "CleanDisconnect";
    case MQTT_CLIENT_CLEAN_RELEASE:
        return "CleanRelease";
    case MQTT_CLIENT_CLEAN_STOP:
        return "CleanStop";
    case MQTT_CLIENT_START:
        return "Start";
    case MQTT_CLIENT_ACQUIRE:
        return "Acquire";
    case MQTT_CLIENT_SSL_VERSION:
        return "SslVersion";
    case MQTT_CLIENT_SSL_AUTHMODE:
        return "SslAuthMode";
    case MQTT_CLIENT_SSL_BIND:
        return "SslBind";
    case MQTT_CLIENT_CONNECT:
        return "Connect";
    case MQTT_CLIENT_CONNECTED:
        return "Connected";
    case MQTT_CLIENT_SET_TOPIC:
        return "SetTopic";
    case MQTT_CLIENT_WRITE_TOPIC:
        return "WriteTopic";
    case MQTT_CLIENT_SET_PAYLOAD:
        return "SetPayload";
    case MQTT_CLIENT_WRITE_PAYLOAD:
        return "WritePayload";
    case MQTT_CLIENT_PUBLISH:
        return "Publish";
    case MQTT_CLIENT_SET_SUBSCRIBE_TOPIC:
        return "SetSubscribeTopic";
    case MQTT_CLIENT_WRITE_SUBSCRIBE_TOPIC:
        return "WriteSubscribeTopic";
    case MQTT_CLIENT_SET_UNSUBSCRIBE_TOPIC:
        return "SetUnsubscribeTopic";
    case MQTT_CLIENT_WRITE_UNSUBSCRIBE_TOPIC:
        return "WriteUnsubscribeTopic";
    case MQTT_CLIENT_FAILED:
        return "Failed";
    default:
        return "Unknown";
    }
}

static bool IsMqttSubscribeState()
{
    return mqttClientState == MQTT_CLIENT_SET_SUBSCRIBE_TOPIC ||
           mqttClientState == MQTT_CLIENT_WRITE_SUBSCRIBE_TOPIC;
}

static bool IsMqttUnsubscribeState()
{
    return mqttClientState == MQTT_CLIENT_SET_UNSUBSCRIBE_TOPIC ||
           mqttClientState == MQTT_CLIENT_WRITE_UNSUBSCRIBE_TOPIC;
}

static bool IsMqttPublishContinuationState()
{
    return mqttClientState > MQTT_CLIENT_SET_TOPIC && mqttClientState <= MQTT_CLIENT_PUBLISH;
}

static bool IsMqttCommandResponseComplete(bool responseError)
{
    if (responseError)
    {
        return true;
    }

    switch (mqttClientState)
    {
    case MQTT_CLIENT_CLEAN_DISCONNECT:
        return HasMqttToken(simBuffer, "+CMQTTDISC:");
    case MQTT_CLIENT_CLEAN_RELEASE:
        return IsSimResponseComplete(simBuffer);
    case MQTT_CLIENT_CLEAN_STOP:
        return HasMqttToken(simBuffer, "+CMQTTSTOP:");
    case MQTT_CLIENT_START:
        return GetMqttSingleResultCode(simBuffer, "+CMQTTSTART:") >= 0;
    case MQTT_CLIENT_ACQUIRE:
    case MQTT_CLIENT_SSL_VERSION:
    case MQTT_CLIENT_SSL_AUTHMODE:
    case MQTT_CLIENT_SSL_BIND:
    case MQTT_CLIENT_WRITE_TOPIC:
    case MQTT_CLIENT_WRITE_PAYLOAD:
        return IsSimResponseComplete(simBuffer);
    case MQTT_CLIENT_CONNECT:
        return GetMqttClientResultCode(simBuffer, "+CMQTTCONNECT:") >= 0;
    case MQTT_CLIENT_SET_TOPIC:
    case MQTT_CLIENT_SET_PAYLOAD:
    case MQTT_CLIENT_SET_SUBSCRIBE_TOPIC:
    case MQTT_CLIENT_SET_UNSUBSCRIBE_TOPIC:
        return HasMqttPrompt(simBuffer);
    case MQTT_CLIENT_PUBLISH:
        return HasMqttToken(simBuffer, "+CMQTTPUB:");
    case MQTT_CLIENT_WRITE_SUBSCRIBE_TOPIC:
        return HasMqttToken(simBuffer, "+CMQTTSUB:");
    case MQTT_CLIENT_WRITE_UNSUBSCRIBE_TOPIC:
        return HasMqttToken(simBuffer, "+CMQTTUNSUB:");
    default:
        return IsSimResponseComplete(simBuffer);
    }
}

static void FailMqttCommand(const char *message, uint32_t now)
{
    mqttConnected = false;
    cellularTestBypassGpsWait = false;
    mqttClientState = MQTT_CLIENT_FAILED;
    if (openRemoteProvisioningActive)
    {
        telemetryBatchActive = false;
        telemetryBatchStartedAt = 0;
    }
    activeTelemetryAttributeIndex = -1;
    mqttSubscribed = false;
    mqttConsecutiveFailureCount++;
    nextMqttPublishAt = now + 5000UL;
    ClearQueuedSimCommand(MQTT_CONNECT);
    ClearQueuedSimCommand(MQTT_PUBLISH);
    ClearQueuedSimCommand(MQTT_SUBSCRIBE);
    ClearQueuedSimCommand(MQTT_UNSUBSCRIBE);
    snprintf(mqttFailureMessage, sizeof(mqttFailureMessage), "%s", message != nullptr ? message : "MQTT command failed.");
}

static bool ShouldQueueSimCommand(SIM7600Commands command)
{
    if (IsCellularDataCommand(command))
    {
        return true;
    }

    if (IsMqttCommand(command))
    {
        return IsMqttConfigured() && dataConnected && internetProbeComplete && internetProbeSucceeded;
    }

    if (command == GPS)
    {
        return true;
    }

    if (command == MODULE_TEMPERATURE)
    {
        return true;
    }

    if (command == SIGNAL_QUALITY)
    {
        return true;
    }

    if (!SystemParams.AllowGPS)
    {
        return true;
    }

    return hasSeenGNSSResponse;
}

static void QueueSimCommand(SIM7600Commands command)
{
    if (!ShouldQueueSimCommand(command))
    {
        return;
    }

    queuedSimCommands |= CommandBit(command);
}

static void ClearQueuedSimCommand(SIM7600Commands command)
{
    queuedSimCommands &= ~CommandBit(command);
}

static bool TryDequeueNextSimCommand(SIM7600Commands *command)
{
    if (command == nullptr)
    {
        return false;
    }

    if ((queuedSimCommands & CommandBit(MQTT_PUBLISH)) != 0 && IsMqttPublishContinuationState())
    {
        ClearQueuedSimCommand(MQTT_PUBLISH);
        *command = MQTT_PUBLISH;
        return true;
    }

    if (SystemParams.AllowGPS && (queuedSimCommands & CommandBit(GPS)) != 0)
    {
        ClearQueuedSimCommand(GPS);
        *command = GPS;
        return true;
    }

    if ((queuedSimCommands & CommandBit(SIGNAL_QUALITY)) != 0)
    {
        ClearQueuedSimCommand(SIGNAL_QUALITY);
        *command = SIGNAL_QUALITY;
        return true;
    }

    if ((queuedSimCommands & CommandBit(CELLULAR_CONNECT)) != 0 &&
        cellularDataState != CELLULAR_DATA_IDLE &&
        cellularDataState != CELLULAR_DATA_CONNECTED &&
        cellularDataState != CELLULAR_DATA_FAILED)
    {
        ClearQueuedSimCommand(CELLULAR_CONNECT);
        *command = CELLULAR_CONNECT;
        return true;
    }

    if ((queuedSimCommands & CommandBit(CELLULAR_DISCONNECT)) != 0)
    {
        ClearQueuedSimCommand(CELLULAR_DISCONNECT);
        *command = CELLULAR_DISCONNECT;
        return true;
    }

    if (SystemParams.AllowGPS && !hasSeenGNSSResponse && !cellularTestBypassGpsWait)
    {
        return false;
    }

    if ((queuedSimCommands & CommandBit(MQTT_CONNECT)) != 0 &&
        mqttClientState != MQTT_CLIENT_IDLE &&
        mqttClientState != MQTT_CLIENT_CONNECTED &&
        mqttClientState != MQTT_CLIENT_FAILED)
    {
        ClearQueuedSimCommand(MQTT_CONNECT);
        *command = MQTT_CONNECT;
        return true;
    }

    if ((queuedSimCommands & CommandBit(MQTT_PUBLISH)) != 0 &&
        mqttClientState >= MQTT_CLIENT_SET_TOPIC &&
        mqttClientState <= MQTT_CLIENT_PUBLISH)
    {
        ClearQueuedSimCommand(MQTT_PUBLISH);
        *command = MQTT_PUBLISH;
        return true;
    }

    if ((queuedSimCommands & CommandBit(MQTT_SUBSCRIBE)) != 0 && IsMqttSubscribeState())
    {
        ClearQueuedSimCommand(MQTT_SUBSCRIBE);
        *command = MQTT_SUBSCRIBE;
        return true;
    }

    if ((queuedSimCommands & CommandBit(MQTT_UNSUBSCRIBE)) != 0 && IsMqttUnsubscribeState())
    {
        ClearQueuedSimCommand(MQTT_UNSUBSCRIBE);
        *command = MQTT_UNSUBSCRIBE;
        return true;
    }

    const SIM7600Commands commandOrder[] = {
        GPS,
        MODULE_TEMPERATURE,
        SIGNAL_QUALITY,
        NETWORK_MODE,
        CELLULAR_CONNECT,
        CELLULAR_DISCONNECT,
        HTTP,
        SMS,
        MQTT,
        MQTT_PUBLISH,
        MQTT_CONNECT,
        MQTT_DISCONNECT,
        MQTT_PING,
        MQTT_STATUS};

    for (SIM7600Commands candidate : commandOrder)
    {
        if ((queuedSimCommands & CommandBit(candidate)) != 0)
        {
            ClearQueuedSimCommand(candidate);
            *command = candidate;
            return true;
        }
    }

    return false;
}

static void ResetSimResponseBuffer()
{
    memset(simBuffer, 0, sizeof(simBuffer));
    simBufferLength = 0;
}

static void CaptureLastCellularModemResponse()
{
    size_t writeIndex = 0;
    for (size_t readIndex = 0; readIndex < simBufferLength && writeIndex < sizeof(lastCellularModemResponse) - 1; readIndex++)
    {
        char character = simBuffer[readIndex];
        if (character == '\r' || character == '\n')
        {
            character = ' ';
        }

        lastCellularModemResponse[writeIndex++] = character;
    }

    lastCellularModemResponse[writeIndex] = '\0';
}

static bool IsSimResponseComplete(const char *response)
{
    return response != nullptr &&
           (strstr(response, "\r\nOK\r\n") != nullptr ||
            strstr(response, "\r\nERROR\r\n") != nullptr ||
            strstr(response, "\nOK\n") != nullptr ||
            strstr(response, "\nERROR\n") != nullptr);
}

static bool HasSimResponseSuccess(const char *response)
{
    return response != nullptr &&
           (strstr(response, "\r\nOK\r\n") != nullptr ||
            strstr(response, "\nOK\n") != nullptr ||
            strstr(response, "DONE") != nullptr ||
            strstr(response, "READY") != nullptr);
}

static bool HasSimResponseError(const char *response)
{
    return response != nullptr &&
           (strstr(response, "\r\nERROR\r\n") != nullptr ||
            strstr(response, "\nERROR\n") != nullptr ||
            strstr(response, "ERROR") != nullptr);
}

static bool HasCompleteGNSSInfoLine(const char *response)
{
    if (response == nullptr)
    {
        return false;
    }

    const char *payload = strstr(response, SIM7600_GNSSINFO_RESPONSE_TOKEN);
    if (payload == nullptr)
    {
        return false;
    }

    payload += strlen(SIM7600_GNSSINFO_RESPONSE_TOKEN);
    return strchr(payload, '\r') != nullptr || strchr(payload, '\n') != nullptr;
}

static bool HasInternetProbeResult(const char *response)
{
    return response != nullptr && strstr(response, "+CDNSGIP:") != nullptr;
}

static bool WasInternetProbeSuccessful(const char *response)
{
    const char *result = response != nullptr ? strstr(response, "+CDNSGIP:") : nullptr;
    if (result == nullptr)
    {
        return false;
    }

    result += strlen("+CDNSGIP:");
    while (*result == ' ' || *result == '\t')
    {
        result++;
    }

    return *result == '1';
}

static bool HasNetworkOpenResult(const char *response)
{
    return response != nullptr &&
           (strstr(response, "+NETOPEN:") != nullptr ||
            HasSimResponseSuccess(response) ||
            HasSimResponseError(response));
}

static bool WasNetworkOpenSuccessful(const char *response)
{
    if (response != nullptr && strstr(response, "Network is already opened") != nullptr)
    {
        return true;
    }

    if (HasSimResponseSuccess(response))
    {
        return true;
    }

    const char *result = response != nullptr ? strstr(response, "+NETOPEN:") : nullptr;
    if (result == nullptr)
    {
        return false;
    }

    result += strlen("+NETOPEN:");
    while (*result == ' ' || *result == '\t')
    {
        result++;
    }

    int errorCode = -1;
    return sscanf(result, "%d", &errorCode) == 1 && errorCode == 0;
}

static bool HasUsablePdpAddress(const char *response)
{
    const char *address = response != nullptr ? strstr(response, "+CGPADDR:") : nullptr;
    if (address == nullptr)
    {
        return false;
    }

    address = strchr(address, ',');
    if (address == nullptr)
    {
        return false;
    }

    address++;
    while (*address == ' ' || *address == '\t' || *address == '"')
    {
        address++;
    }

    return *address != '\0' && *address != '\r' && *address != '\n' && strstr(address, "0.0.0.0") != address;
}

static void SendGPSPowerCommand(bool enableGPS, uint32_t now)
{
    ResetSimResponseBuffer();
    if (enableGPS)
    {
        Serial1.print("AT+CGPS=1,1\r");
    }
    else
    {
        Serial1.print("AT+CGPS=0\r");
        ResetGPSPlausibilityFilter();
        ClearQueuedSimCommand(GPS);
    }

    activeSimCommandType = SIM_ACTIVE_COMMAND_GPS_POWER;
    activeGPSEnableState = enableGPS;
    simCommandPending = true;
    simCommandSentAt = now;
}

static void SendSimCommand(SIM7600Commands command)
{
    char commandBuffer[256];

    switch (command)
    {
    case GPS:
        if (SystemParams.AllowGPS)
        {
            Serial1.print("AT+CGNSSINFO\r");
#ifdef DEBUG
            Serial.println("Requesting GPS info...");
#endif
        }
        break;
    case HTTP:
        break;
    case SMS:
        break;
    case MQTT:
        break;
    case MQTT_SUBSCRIBE:
        switch (mqttClientState)
        {
        case MQTT_CLIENT_SET_SUBSCRIBE_TOPIC:
            if (mqttTopicBuffer[0] != '\0')
            {
                snprintf(commandBuffer, sizeof(commandBuffer), "AT+CMQTTSUB=0,%u,1\r", (unsigned int)strlen(mqttTopicBuffer));
                Serial1.print(commandBuffer);
            }
            break;
        case MQTT_CLIENT_WRITE_SUBSCRIBE_TOPIC:
            Serial1.print(mqttTopicBuffer);
            break;
        default:
            break;
        }
        break;
    case MQTT_UNSUBSCRIBE:
        switch (mqttClientState)
        {
        case MQTT_CLIENT_SET_UNSUBSCRIBE_TOPIC:
            if (mqttTopicBuffer[0] != '\0')
            {
                snprintf(commandBuffer, sizeof(commandBuffer), "AT+CMQTTUNSUB=0,%u\r", (unsigned int)strlen(mqttTopicBuffer));
                Serial1.print(commandBuffer);
            }
            break;
        case MQTT_CLIENT_WRITE_UNSUBSCRIBE_TOPIC:
            Serial1.print(mqttTopicBuffer);
            break;
        default:
            break;
        }
        break;
    case MQTT_DISCONNECT:
        break;
    case MQTT_PING:
        break;
    case MQTT_STATUS:
        break;
    case MQTT_CONNECT:
        switch (mqttClientState)
        {
        case MQTT_CLIENT_CLEAN_DISCONNECT:
            Serial1.print("AT+CMQTTDISC=0,60\r");
            break;
        case MQTT_CLIENT_CLEAN_RELEASE:
            Serial1.print("AT+CMQTTREL=0\r");
            break;
        case MQTT_CLIENT_CLEAN_STOP:
            Serial1.print("AT+CMQTTSTOP\r");
            break;
        case MQTT_CLIENT_START:
            Serial1.print("AT+CMQTTSTART\r");
            break;
        case MQTT_CLIENT_ACQUIRE:
        {
            const char *mqttConnectClientID = GetMqttConnectClientID();
            snprintf(lastMqttConnectClientID, sizeof(lastMqttConnectClientID), "%s", mqttConnectClientID);
            snprintf(commandBuffer, sizeof(commandBuffer), "AT+CMQTTACCQ=0,\"%s\",%u,4\r", mqttConnectClientID, CellularParams.UseTLS ? 1U : 0U);
            Serial1.print(commandBuffer);
            break;
        }
        case MQTT_CLIENT_SSL_VERSION:
            Serial1.print("AT+CSSLCFG=\"sslversion\",0,3\r");
            break;
        case MQTT_CLIENT_SSL_AUTHMODE:
            Serial1.print("AT+CSSLCFG=\"authmode\",0,0\r");
            break;
        case MQTT_CLIENT_SSL_BIND:
            Serial1.print("AT+CMQTTSSLCFG=0,0\r");
            break;
        case MQTT_CLIENT_CONNECT:
            if (openRemoteProvisioningActive)
            {
                snprintf(lastMqttConnectAuthMode, sizeof(lastMqttConnectAuthMode), "none");
                snprintf(commandBuffer,
                         sizeof(commandBuffer),
                         "AT+CMQTTCONNECT=0,\"%s://%s:%u\",%u,1\r",
                         "tcp",
                         CellularParams.OpenRemoteHost,
                         CellularParams.OpenRemotePort,
                         CellularParams.KeepAliveSeconds == 0 ? CELLULAR_DEFAULT_KEEPALIVE_SECONDS : CellularParams.KeepAliveSeconds);
            }
            else
            {
                snprintf(lastMqttConnectAuthMode, sizeof(lastMqttConnectAuthMode), "userpass");
                const char *mqttUsername = GetMqttConnectUsername();
                snprintf(commandBuffer,
                         sizeof(commandBuffer),
                         "AT+CMQTTCONNECT=0,\"%s://%s:%u\",%u,1,\"%s\",\"%s\"\r",
                         "tcp",
                         CellularParams.OpenRemoteHost,
                         CellularParams.OpenRemotePort,
                         CellularParams.KeepAliveSeconds == 0 ? CELLULAR_DEFAULT_KEEPALIVE_SECONDS : CellularParams.KeepAliveSeconds,
                         mqttUsername,
                         CellularParams.MQTTPassword);
            }
            Serial1.print(commandBuffer);
            snprintf(lastMqttModemResponse, sizeof(lastMqttModemResponse), "AT+CMQTTCONNECT sent; waiting for modem result.");
            break;
        default:
            break;
        }
        break;
    case MQTT_PUBLISH:
        switch (mqttClientState)
        {
        case MQTT_CLIENT_SET_TOPIC:
            if (mqttTopicBuffer[0] != '\0')
            {
                snprintf(commandBuffer, sizeof(commandBuffer), "AT+CMQTTTOPIC=0,%u\r", (unsigned int)strlen(mqttTopicBuffer));
                Serial1.print(commandBuffer);
            }
            break;
        case MQTT_CLIENT_WRITE_TOPIC:
            Serial1.print(mqttTopicBuffer);
            break;
        case MQTT_CLIENT_SET_PAYLOAD:
            snprintf(commandBuffer, sizeof(commandBuffer), "AT+CMQTTPAYLOAD=0,%u\r", (unsigned int)strlen(GetActiveMqttPayload()));
            Serial1.print(commandBuffer);
            break;
        case MQTT_CLIENT_WRITE_PAYLOAD:
            Serial1.print(GetActiveMqttPayload());
            break;
        case MQTT_CLIENT_PUBLISH:
            Serial1.print("AT+CMQTTPUB=0,0,60\r");
            break;
        default:
            break;
        }
        break;
    case CELLULAR_CONNECT:
        switch (cellularDataState)
        {
        case CELLULAR_DATA_SET_APN:
            snprintf(commandBuffer, sizeof(commandBuffer), "AT+CGDCONT=1,\"IP\",\"%s\"\r", CellularParams.APN);
            Serial1.print(commandBuffer);
            break;
        case CELLULAR_DATA_ATTACH:
            Serial1.print("AT+CGATT=1\r");
            break;
        case CELLULAR_DATA_ACTIVATE_PDP:
            Serial1.print("AT+CGACT=1,1\r");
            break;
        case CELLULAR_DATA_CHECK_ADDRESS:
            Serial1.print("AT+CGPADDR=1\r");
            break;
        case CELLULAR_DATA_OPEN_NETWORK:
            Serial1.print("AT+NETOPEN\r");
            break;
        case CELLULAR_DATA_SET_DNS:
            Serial1.print("AT+CDNSCFG=\"1.1.1.1\",\"8.8.8.8\"\r");
            break;
        case CELLULAR_DATA_PROBE_INTERNET:
            snprintf(commandBuffer, sizeof(commandBuffer), "AT+CDNSGIP=\"%s\"\r", GetInternetProbeHost());
            Serial1.print(commandBuffer);
            break;
        default:
            break;
        }
        break;
    case CELLULAR_DISCONNECT:
        Serial1.print("AT+CGACT=0,1\r");
        break;
    case SIGNAL_QUALITY:
        Serial1.print("AT+CSQ\r");
        break;
    case NETWORK_MODE:
        Serial1.print("AT+CESQ?\r");
        break;
    case MODULE_TEMPERATURE:
        Serial1.print("AT+CPMUTEMP\r");
        break;
    }
}

static void ProcessCellularDataCommandResult(bool responseSuccess, bool responseError, bool commandTimedOut, uint32_t now)
{
    bool commandFailed = commandTimedOut || responseError || !responseSuccess;

    if (pendingCommand == CELLULAR_DISCONNECT)
    {
        dataConnected = false;
        cellularDataState = CELLULAR_DATA_IDLE;
        nextCellularDataAttemptAt = now + SIM7600_DATA_RETRY_MS;
        return;
    }

    if (pendingCommand != CELLULAR_CONNECT)
    {
        return;
    }

    if (cellularDataState == CELLULAR_DATA_PROBE_INTERNET)
    {
        internetProbeComplete = true;
        internetProbeSucceeded = !commandTimedOut && WasInternetProbeSuccessful(simBuffer);
        cellularDataState = CELLULAR_DATA_CONNECTED;
        nextCellularDataAttemptAt = now + SIM7600_DATA_RETRY_MS;
        return;
    }

    if (cellularDataState == CELLULAR_DATA_OPEN_NETWORK)
    {
        if (commandTimedOut || !WasNetworkOpenSuccessful(simBuffer))
        {
            internetProbeComplete = true;
            internetProbeSucceeded = false;
            cellularDataState = CELLULAR_DATA_CONNECTED;
            nextCellularDataAttemptAt = now + SIM7600_DATA_RETRY_MS;
            return;
        }

        cellularDataState = CELLULAR_DATA_PROBE_INTERNET;
        QueueSimCommand(CELLULAR_CONNECT);
        return;
    }

    if (cellularDataState == CELLULAR_DATA_SET_DNS)
    {
        cellularDataState = CELLULAR_DATA_PROBE_INTERNET;
        QueueSimCommand(CELLULAR_CONNECT);
        return;
    }

    if (commandFailed)
    {
        if (cellularDataState == CELLULAR_DATA_SET_APN && responseError)
        {
            cellularDataState = CELLULAR_DATA_ATTACH;
            QueueSimCommand(CELLULAR_CONNECT);
            return;
        }

        if (cellularDataState == CELLULAR_DATA_ATTACH && commandTimedOut)
        {
            cellularDataState = CELLULAR_DATA_ACTIVATE_PDP;
            QueueSimCommand(CELLULAR_CONNECT);
            return;
        }

        if (cellularDataState == CELLULAR_DATA_ACTIVATE_PDP && responseError)
        {
            cellularDataState = CELLULAR_DATA_CHECK_ADDRESS;
            QueueSimCommand(CELLULAR_CONNECT);
            return;
        }

        const char *stage = "Data command";
        switch (cellularDataState)
        {
        case CELLULAR_DATA_SET_APN:
            stage = "APN setup";
            break;
        case CELLULAR_DATA_ATTACH:
            stage = "Packet data attach";
            break;
        case CELLULAR_DATA_ACTIVATE_PDP:
            stage = "PDP activation";
            break;
        case CELLULAR_DATA_CHECK_ADDRESS:
            stage = "IP address check";
            break;
        case CELLULAR_DATA_OPEN_NETWORK:
            stage = "Network open";
            break;
        default:
            break;
        }

        FailCellularDataTest(stage, responseError, commandTimedOut);
        return;
    }

    switch (cellularDataState)
    {
    case CELLULAR_DATA_SET_APN:
        cellularDataState = CELLULAR_DATA_ATTACH;
        QueueSimCommand(CELLULAR_CONNECT);
        break;
    case CELLULAR_DATA_ATTACH:
        cellularDataState = CELLULAR_DATA_ACTIVATE_PDP;
        QueueSimCommand(CELLULAR_CONNECT);
        break;
    case CELLULAR_DATA_ACTIVATE_PDP:
        cellularDataState = CELLULAR_DATA_CHECK_ADDRESS;
        QueueSimCommand(CELLULAR_CONNECT);
        break;
    case CELLULAR_DATA_CHECK_ADDRESS:
        if (!HasUsablePdpAddress(simBuffer))
        {
            FailCellularDataTest("IP address check", responseError, commandTimedOut);
            return;
        }

        dataConnected = true;
        cellularDataState = CELLULAR_DATA_OPEN_NETWORK;
        QueueSimCommand(CELLULAR_CONNECT);
        break;
    case CELLULAR_DATA_OPEN_NETWORK:
        cellularDataState = CELLULAR_DATA_SET_DNS;
        QueueSimCommand(CELLULAR_CONNECT);
        break;
    case CELLULAR_DATA_SET_DNS:
        cellularDataState = CELLULAR_DATA_PROBE_INTERNET;
        QueueSimCommand(CELLULAR_CONNECT);
        break;
    case CELLULAR_DATA_PROBE_INTERNET:
        internetProbeComplete = true;
        internetProbeSucceeded = WasInternetProbeSuccessful(simBuffer);
        cellularDataState = CELLULAR_DATA_CONNECTED;
        nextCellularDataAttemptAt = now + SIM7600_DATA_RETRY_MS;
        break;
    default:
        ScheduleCellularDataRetry(now);
        break;
    }
}

static void StartMqttConnection()
{
    mqttConnected = false;
    mqttSubscribed = false;
    mqttFailureMessage[0] = '\0';
    lastMqttModemResponse[0] = '\0';
    mqttClientState = MQTT_CLIENT_CLEAN_DISCONNECT;
    QueueSimCommand(MQTT_CONNECT);
}

static bool StartMqttSubscribe(uint32_t now)
{
    if (openRemoteProvisioningActive)
    {
        if (!BuildOpenRemoteProvisioningResponseTopic())
        {
            return false;
        }
    }
    else if (CellularParams.SubscribeTopic[0] == '\0')
    {
        return false;
    }
    else
    {
        snprintf(mqttTopicBuffer, sizeof(mqttTopicBuffer), "%s", CellularParams.SubscribeTopic);
    }

    lastMqttSubscribeAttemptAt = now;
    mqttClientState = MQTT_CLIENT_SET_SUBSCRIBE_TOPIC;
    QueueSimCommand(MQTT_SUBSCRIBE);
    return true;
}

static bool StartMqttPublish(uint32_t now)
{
    mqttFailureMessage[0] = '\0';
    mqttTopicBuffer[0] = '\0';
    mqttPayloadBuffer[0] = '\0';
    if (openRemoteProvisioningActive)
    {
        int topicWritten = snprintf(mqttTopicBuffer,
                                    sizeof(mqttTopicBuffer),
                                    "provisioning/%s/request",
                                    CellularParams.ClientID);
        if (topicWritten <= 0 || static_cast<size_t>(topicWritten) >= sizeof(mqttTopicBuffer))
        {
            snprintf(openRemoteProvisioningStatus, sizeof(openRemoteProvisioningStatus), "Provisioning topic is too long.");
            openRemoteProvisioningComplete = true;
            openRemoteProvisioningSuccess = false;
            return false;
        }

        mqttPublishAttemptCount++;
        lastMqttPublishAttemptAt = now;
        activeTelemetryAttributeIndex = -1;
    }
    else if (!PrepareNextTelemetryPublish(now))
    {
        return false;
    }

    mqttClientState = MQTT_CLIENT_SET_TOPIC;
    QueueSimCommand(MQTT_PUBLISH);
    return true;
}

static void ProcessMqttConnectCommandResult(bool responseError, bool commandTimedOut, uint32_t now)
{
    if (mqttClientState == MQTT_CLIENT_CLEAN_DISCONNECT)
    {
        mqttClientState = MQTT_CLIENT_CLEAN_RELEASE;
        QueueSimCommand(MQTT_CONNECT);
        return;
    }

    if (mqttClientState == MQTT_CLIENT_CLEAN_RELEASE)
    {
        mqttClientState = MQTT_CLIENT_CLEAN_STOP;
        QueueSimCommand(MQTT_CONNECT);
        return;
    }

    if (mqttClientState == MQTT_CLIENT_CLEAN_STOP)
    {
        mqttClientState = MQTT_CLIENT_START;
        QueueSimCommand(MQTT_CONNECT);
        return;
    }

    if (commandTimedOut || responseError)
    {
        FailMqttCommand(commandTimedOut ? "MQTT connect command timed out." : "MQTT connect command returned ERROR.", now);
        return;
    }

    switch (mqttClientState)
    {
    case MQTT_CLIENT_START:
        if (!WasMqttSingleCodeSuccessful(simBuffer, "+CMQTTSTART:"))
        {
            int resultCode = GetMqttSingleResultCode(simBuffer, "+CMQTTSTART:");
            if (resultCode != 23)
            {
                char failureMessage[96];
                if (resultCode >= 0)
                {
                    snprintf(failureMessage, sizeof(failureMessage), "MQTT service did not start with code %d.", resultCode);
                }
                else
                {
                    snprintf(failureMessage, sizeof(failureMessage), "MQTT service did not return a complete start result.");
                }
                FailMqttCommand(failureMessage, now);
                return;
            }
        }
        mqttClientState = MQTT_CLIENT_ACQUIRE;
        QueueSimCommand(MQTT_CONNECT);
        break;
    case MQTT_CLIENT_ACQUIRE:
        mqttClientState = CellularParams.UseTLS ? MQTT_CLIENT_SSL_VERSION : MQTT_CLIENT_CONNECT;
        QueueSimCommand(MQTT_CONNECT);
        break;
    case MQTT_CLIENT_SSL_VERSION:
        mqttClientState = MQTT_CLIENT_SSL_AUTHMODE;
        QueueSimCommand(MQTT_CONNECT);
        break;
    case MQTT_CLIENT_SSL_AUTHMODE:
        mqttClientState = MQTT_CLIENT_SSL_BIND;
        QueueSimCommand(MQTT_CONNECT);
        break;
    case MQTT_CLIENT_SSL_BIND:
        mqttClientState = MQTT_CLIENT_CONNECT;
        QueueSimCommand(MQTT_CONNECT);
        break;
    case MQTT_CLIENT_CONNECT:
        if (!WasMqttClientCodeSuccessful(simBuffer, "+CMQTTCONNECT:"))
        {
            int resultCode = GetMqttClientResultCode(simBuffer, "+CMQTTCONNECT:");
            if (resultCode >= 0)
            {
                char failureMessage[96];
                if (resultCode == 6)
                {
                    snprintf(failureMessage, sizeof(failureMessage), "MQTT connect failed with modem code 6: message receive failed.");
                }
                else if (resultCode == 28)
                {
                    snprintf(failureMessage, sizeof(failureMessage), "MQTT broker rejected the client ID.");
                }
                else if (resultCode == 30 && IsOpenRemotePublishTopic() && !IsOpenRemoteMqttUsernameRealmPrefixed())
                {
                    snprintf(failureMessage, sizeof(failureMessage), "MQTT broker refused login. Use realm:service-user for MQTT username.");
                }
                else if (resultCode == 30)
                {
                    snprintf(failureMessage, sizeof(failureMessage), "MQTT broker refused the username or password.");
                }
                else if (resultCode == 31)
                {
                    snprintf(failureMessage, sizeof(failureMessage), "MQTT broker refused authorization for this client.");
                }
                else if (resultCode == 22 && openRemoteProvisioningActive)
                {
                    snprintf(failureMessage, sizeof(failureMessage), "Provisioning MQTT was refused. Check anonymous MQTT/provisioning listener.");
                }
                else
                {
                    snprintf(failureMessage, sizeof(failureMessage), "MQTT broker connection failed with code %d.", resultCode);
                }
                FailMqttCommand(failureMessage, now);
            }
            else
            {
                FailMqttCommand("MQTT broker connection failed.", now);
            }
            return;
        }
        mqttConnected = true;
        if (!mqttSubscribed &&
            ((openRemoteProvisioningActive && StartMqttSubscribe(now)) ||
             (!openRemoteProvisioningActive && CellularParams.SubscribeTopic[0] != '\0' && StartMqttSubscribe(now))))
        {
            break;
        }

        mqttClientState = MQTT_CLIENT_CONNECTED;
        nextMqttPublishAt = now;
        break;
    default:
        char failureMessage[96];
        snprintf(failureMessage, sizeof(failureMessage), "Unexpected MQTT connect state: %s.", GetMqttClientStateName());
        FailMqttCommand(failureMessage, now);
        break;
    }
}

static bool WaitForActiveMqttCommandToFinish(uint32_t deadline)
{
    while (simCommandPending && IsMqttCommand(pendingCommand) && (int32_t)(millis() - deadline) < 0)
    {
        UpdateSIM7600();
        IWatchdog.reload();
        delay(5);
    }

    return !(simCommandPending && IsMqttCommand(pendingCommand));
}

static void ProcessMqttPublishCommandResult(bool responseError, bool commandTimedOut, uint32_t now)
{
    if (commandTimedOut || responseError)
    {
        RetryActiveTelemetryAttributeOnce();
        FailMqttCommand(commandTimedOut ? "MQTT publish command timed out." : "MQTT publish command returned ERROR.", now);
        return;
    }

    switch (mqttClientState)
    {
    case MQTT_CLIENT_SET_TOPIC:
        mqttClientState = MQTT_CLIENT_WRITE_TOPIC;
        QueueSimCommand(MQTT_PUBLISH);
        break;
    case MQTT_CLIENT_WRITE_TOPIC:
        mqttClientState = MQTT_CLIENT_SET_PAYLOAD;
        QueueSimCommand(MQTT_PUBLISH);
        break;
    case MQTT_CLIENT_SET_PAYLOAD:
        mqttClientState = MQTT_CLIENT_WRITE_PAYLOAD;
        QueueSimCommand(MQTT_PUBLISH);
        break;
    case MQTT_CLIENT_WRITE_PAYLOAD:
        mqttClientState = MQTT_CLIENT_PUBLISH;
        QueueSimCommand(MQTT_PUBLISH);
        break;
    case MQTT_CLIENT_PUBLISH:
        if (!WasMqttClientCodeSuccessful(simBuffer, "+CMQTTPUB:"))
        {
            RetryActiveTelemetryAttributeOnce();
            FailMqttCommand("MQTT publish was rejected.", now);
            return;
        }
        mqttPublishSuccessCount++;
        mqttConsecutiveFailureCount = 0;
        lastMqttPublishSuccessAt = now;
        if (openRemoteProvisioningActive)
        {
            snprintf(lastMqttPublishAttribute, sizeof(lastMqttPublishAttribute), "OpenRemoteProvisioning");
            snprintf(lastMqttPublishPayload, sizeof(lastMqttPublishPayload), "request published");
            openRemoteProvisioningRequestPublished = true;
            snprintf(openRemoteProvisioningStatus, sizeof(openRemoteProvisioningStatus), "OpenRemote provisioning request was published; waiting for response.");
            mqttClientState = MQTT_CLIENT_CONNECTED;
            nextMqttPublishAt = now + OPENREMOTE_PROVISIONING_TIMEOUT_MS;
            break;
        }

        if (activeTelemetryAttributeIndex >= 0 && activeTelemetryAttributeIndex < static_cast<int8_t>(TELEMETRY_ATTRIBUTE_COUNT))
        {
            ClearTelemetryAttributeRetry(activeTelemetryAttributeIndex);
            const char *attributeName = TELEMETRY_ATTRIBUTES[activeTelemetryAttributeIndex].attributeName;
            snprintf(lastMqttPublishAttribute, sizeof(lastMqttPublishAttribute), "%s", attributeName);
            snprintf(lastMqttPublishPayload, sizeof(lastMqttPublishPayload), "%s", GetActiveMqttPayload());
            if (strcmp(attributeName, "location") == 0)
            {
                lastLocationPublishSuccessAt = now;
                snprintf(lastLocationPublishPayload, sizeof(lastLocationPublishPayload), "%s", mqttPayloadBuffer);
            }
        }
        mqttClientState = MQTT_CLIENT_CONNECTED;
        activeTelemetryAttributeIndex = -1;
        nextMqttPublishAt = telemetryBatchActive ? now : now + GetMqttPublishIntervalMs();
        break;
    default:
        FailMqttCommand("Unexpected MQTT publish state.", now);
        break;
    }
}

static void ProcessMqttSubscribeCommandResult(bool responseError, bool commandTimedOut, uint32_t now)
{
    if (commandTimedOut || responseError)
    {
        FailMqttCommand(commandTimedOut ? "MQTT subscribe command timed out." : "MQTT subscribe command returned ERROR.", now);
        return;
    }

    switch (mqttClientState)
    {
    case MQTT_CLIENT_SET_SUBSCRIBE_TOPIC:
        mqttClientState = MQTT_CLIENT_WRITE_SUBSCRIBE_TOPIC;
        QueueSimCommand(MQTT_SUBSCRIBE);
        break;
    case MQTT_CLIENT_WRITE_SUBSCRIBE_TOPIC:
        if (!WasMqttClientCodeSuccessful(simBuffer, "+CMQTTSUB:"))
        {
            FailMqttCommand("MQTT subscribe was rejected.", now);
            return;
        }
        mqttSubscribed = true;
        lastMqttSubscribeSuccessAt = now;
        snprintf(lastMqttSubscribeTopic, sizeof(lastMqttSubscribeTopic), "%s", mqttTopicBuffer);
        if (openRemoteProvisioningActive)
        {
            snprintf(openRemoteProvisioningStatus, sizeof(openRemoteProvisioningStatus), "Subscribed to OpenRemote provisioning response.");
        }
        mqttClientState = MQTT_CLIENT_CONNECTED;
        nextMqttPublishAt = now;
        break;
    default:
        FailMqttCommand("Unexpected MQTT subscribe state.", now);
        break;
    }
}

static void ProcessMqttUnsubscribeCommandResult(bool responseError, bool commandTimedOut, uint32_t now)
{
    if (commandTimedOut || responseError)
    {
        FailMqttCommand(commandTimedOut ? "MQTT unsubscribe command timed out." : "MQTT unsubscribe command returned ERROR.", now);
        return;
    }

    switch (mqttClientState)
    {
    case MQTT_CLIENT_SET_UNSUBSCRIBE_TOPIC:
        mqttClientState = MQTT_CLIENT_WRITE_UNSUBSCRIBE_TOPIC;
        QueueSimCommand(MQTT_UNSUBSCRIBE);
        break;
    case MQTT_CLIENT_WRITE_UNSUBSCRIBE_TOPIC:
        if (!WasMqttClientCodeSuccessful(simBuffer, "+CMQTTUNSUB:"))
        {
            FailMqttCommand("MQTT unsubscribe was rejected.", now);
            return;
        }
        mqttSubscribed = false;
        lastMqttSubscribeTopic[0] = '\0';
        mqttClientState = MQTT_CLIENT_CONNECTED;
        nextMqttPublishAt = now;
        break;
    default:
        FailMqttCommand("Unexpected MQTT unsubscribe state.", now);
        break;
    }
}

static bool TryParseSimTemperatureResponse(const char *response, float *temperature)
{
    if (response == nullptr || temperature == nullptr)
    {
        return false;
    }

    const char *token = strstr(response, SIM7600_TEMP_RESPONSE_TOKEN);
    if (token == nullptr)
    {
        return false;
    }

    token += strlen(SIM7600_TEMP_RESPONSE_TOKEN);
    while (*token != '\0' && (*token == ':' || *token == '=' || *token == ' ' || *token == '\t' || *token == '"'))
    {
        token++;
    }

    while (*token != '\0' && !isdigit(static_cast<unsigned char>(*token)) && *token != '-' && *token != '+')
    {
        token++;
    }

    if (*token == '\0')
    {
        return false;
    }

    char *endPtr = nullptr;
    float parsedTemperature = strtof(token, &endPtr);
    if (endPtr == token)
    {
        return false;
    }

    *temperature = parsedTemperature;
    return true;
}

static void ProcessSimResponseBuffer()
{
    if (HasSimResponseSuccess(simBuffer))
    {
        if (SIM7600State == 0)
        {
            SIM7600State = 1;
        }
        else if (SIM7600State == 1)
        {
            SIM7600State = 2;
        }
    }

    if (strstr(simBuffer, "ERROR") != nullptr)
    {
        SIM7600State = 2;
    }

    if (HasCompleteGNSSInfoLine(simBuffer))
    {
        parseGPSData(simBuffer);
        SIM7600State = 2;
    }

    if (strstr(simBuffer, "+CSQ:") != nullptr)
    {
        char *csq = strstr(simBuffer, "+CSQ:");
        if (csq != nullptr)
        {
            int parsedRssi = SIM7600_CSQ_RSSI_UNKNOWN;
            int ber = 0;
            if (sscanf(csq, "+CSQ: %d,%d", &parsedRssi, &ber) == 2 && parsedRssi >= 0 && parsedRssi <= SIM7600_CSQ_RSSI_UNKNOWN)
            {
                rssi = parsedRssi;
            }
            else
            {
                rssi = SIM7600_CSQ_RSSI_UNKNOWN;
            }
        }
#ifdef DEBUG
        Serial.print("Signal RSSI: ");
        Serial.println(rssi);
#endif
    }

    char *cesq = strstr(simBuffer, "+CESQ:");
    if (cesq != nullptr)
    {
        int rxlev = 0, ber = 0, rscp = 0, ecio = 0, rsrq = 0, rsrp = 0;

        if (sscanf(cesq, "+CESQ: %d,%d,%d,%d,%d,%d",
                   &rxlev, &ber, &rscp, &ecio, &rsrq, &rsrp) == 6)
        {
#ifdef DEBUG
            Serial.print("CESQ rxlev: ");
            Serial.println(rxlev);
            Serial.print("CESQ ber: ");
            Serial.println(ber);
            Serial.print("CESQ rscp: ");
            Serial.println(rscp);
            Serial.print("CESQ ecio: ");
            Serial.println(ecio);
            Serial.print("CESQ rsrq: ");
            Serial.println(rsrq);
            Serial.print("CESQ rsrp: ");
            Serial.println(rsrp);
#endif
        }
    }

    float parsedTemperature = 0.0f;
    if (TryParseSimTemperatureResponse(simBuffer, &parsedTemperature))
    {
        simModuleTemp = parsedTemperature;
    }
}

void InitialiseGSM()
{
    delay(SIM7600_REGULATOR_STABLE_MS);

    pinMode(SIM_PWR, OUTPUT);
    pinMode(SIM_RST, OUTPUT);
    pinMode(SIM_FLIGHT, OUTPUT);

    digitalWrite(SIM_PWR, LOW);
    digitalWrite(SIM_RST, LOW);
    digitalWrite(SIM_FLIGHT, LOW);

    delay(SIM7600_POWER_KEY_SETTLE_MS);

    // Power the module on
    digitalWrite(SIM_PWR, HIGH);
    Serial1.begin(GSM_BAUD_RATE);
    delay(10);
    while (Serial1.available())
    {
        Serial1.read();
    }
    simStartupDeadline = millis() + SIM7600_BOOT_WAIT_MS;
    nextGPSEnableRetryAt = simStartupDeadline;
    ResetAcceptedGPSFixState();
    GPSFix = false;
    simModuleTemp = 0.0f;
    rssi = SIM7600_CSQ_RSSI_UNKNOWN;
    previousGPSEnable = false;
    gpsEnableAcceptedAt = 0;
    SIM7600State = 0;
    simCommandPending = false;
    pendingCommand = GPS;
    simCommandSentAt = 0;
    queuedSimCommands = 0;
    activeSimCommandType = SIM_ACTIVE_COMMAND_NONE;
    activeGPSEnableState = false;
    cellularDataState = CELLULAR_DATA_IDLE;
    nextCellularDataAttemptAt = simStartupDeadline;
    cellularTestBypassGpsWait = false;
    internetProbeComplete = false;
    internetProbeSucceeded = false;
    cellularDataFailureMessage[0] = '\0';
    mqttClientState = MQTT_CLIENT_IDLE;
    mqttConnected = false;
    nextMqttPublishAt = 0;
    mqttFailureMessage[0] = '\0';
    lastMqttModemResponse[0] = '\0';
    mqttPublishAttemptCount = 0;
    mqttPublishSuccessCount = 0;
    mqttConsecutiveFailureCount = 0;
    lastMqttPublishAttemptAt = 0;
    lastMqttPublishSuccessAt = 0;
    telemetryHealthMonitoringSince = 0;
    lastMqttSubscribeAttemptAt = 0;
    lastMqttSubscribeSuccessAt = 0;
    lastMqttPublishPayload[0] = '\0';
    lastMqttPublishAttribute[0] = '\0';
    lastMqttSubscribeTopic[0] = '\0';
    mqttSubscribed = false;
    lastLocationPublishSuccessAt = 0;
    lastLocationPublishPayload[0] = '\0';
    telemetryBatchActive = false;
    telemetryBatchCursor = 0;
    telemetryBatchStartedAt = 0;
    activeTelemetryAttributeIndex = -1;
    openRemoteProvisioningActive = false;
    openRemoteProvisioningComplete = false;
    openRemoteProvisioningSuccess = false;
    openRemoteProvisioningRequestPublished = false;
    openRemoteProvisioningRequest[0] = '\0';
    openRemoteProvisioningStatus[0] = '\0';
    openRemoteProvisioningRealm[0] = '\0';
    openRemoteProvisioningAssetID[0] = '\0';
    ResetOpenRemoteProvisioningReceiveCapture();
    dataConnected = false;
    ResetSimResponseBuffer();
#ifdef DEBUG
    Serial.println("Initializing SIM7600G...");
#endif
}

void ResetGPSPlausibilityFilter()
{
    ResetAcceptedGPSFixState();
    GPSFix = false;
}

void RequestCellularConnectionTest()
{
    uint32_t now = millis();

    if (!IsCellularDataEnabled() || !CanStartCellularDataConnection() || cellularDataState == CELLULAR_DATA_FAILED)
    {
        return;
    }

    cellularTestBypassGpsWait = true;

    if (dataConnected && cellularDataState == CELLULAR_DATA_CONNECTED)
    {
        return;
    }

    if (cellularDataState == CELLULAR_DATA_IDLE)
    {
        StartCellularDataConnection(now);
    }
}

void ResetCellularConnectionTest()
{
    if (simCommandPending)
    {
        return;
    }

    bool preserveDataConnection = dataConnected && cellularDataState == CELLULAR_DATA_CONNECTED && internetProbeComplete && internetProbeSucceeded;
    bool preserveMqttConnection = preserveDataConnection && mqttConnected && mqttClientState == MQTT_CLIENT_CONNECTED;

    if (!preserveDataConnection)
    {
        dataConnected = false;
        internetProbeComplete = false;
        internetProbeSucceeded = false;
        cellularDataState = CELLULAR_DATA_IDLE;
    }

    cellularDataFailureMessage[0] = '\0';
    lastCellularModemResponse[0] = '\0';

    if (!preserveMqttConnection)
    {
        mqttClientState = MQTT_CLIENT_IDLE;
        mqttConnected = false;
        mqttSubscribed = false;
        nextMqttPublishAt = 0;
    }

    mqttFailureMessage[0] = '\0';
    lastMqttModemResponse[0] = '\0';
    cellularTestBypassGpsWait = false;
    ClearQueuedSimCommand(CELLULAR_CONNECT);
    ClearQueuedSimCommand(CELLULAR_DISCONNECT);
    ClearQueuedSimCommand(MQTT_CONNECT);
    ClearQueuedSimCommand(MQTT_PUBLISH);
    ClearQueuedSimCommand(MQTT_SUBSCRIBE);
    ClearQueuedSimCommand(MQTT_UNSUBSCRIBE);
}

bool RunOpenRemoteProvisioningRequest(const char *requestJson, size_t requestLength, char *resultBuffer, size_t resultBufferSize)
{
    if (resultBuffer != nullptr && resultBufferSize > 0)
    {
        resultBuffer[0] = '\0';
    }

    if (requestJson == nullptr || requestLength == 0 || requestLength > OPENREMOTE_PROVISIONING_MAX_REQUEST_BYTES)
    {
        if (resultBuffer != nullptr && resultBufferSize > 0)
        {
            snprintf(resultBuffer, resultBufferSize, "Provisioning request is empty or too large.");
        }
        return false;
    }

    if (SystemParams.AllowData == 0 || CellularParams.APN[0] == '\0' || CellularParams.OpenRemoteHost[0] == '\0' || CellularParams.ClientID[0] == '\0')
    {
        if (resultBuffer != nullptr && resultBufferSize > 0)
        {
            snprintf(resultBuffer, resultBufferSize, "Mobile data, APN, MQTT host, and client ID must be configured before provisioning.");
        }
        return false;
    }

    if (CellularParams.UseTLS == 0)
    {
        if (resultBuffer != nullptr && resultBufferSize > 0)
        {
            snprintf(resultBuffer, resultBufferSize, "OpenRemote provisioning requires TLS. Enable TLS and use port %u unless your broker uses a custom TLS port.", CELLULAR_DEFAULT_MQTT_TLS_PORT);
        }
        return false;
    }

    if (simCommandPending && IsMqttCommand(pendingCommand))
    {
        uint32_t quiesceDeadline = millis() + GetActiveSimCommandTimeoutMs() + 1000UL;
        if (!WaitForActiveMqttCommandToFinish(quiesceDeadline))
        {
            if (resultBuffer != nullptr && resultBufferSize > 0)
            {
                snprintf(resultBuffer,
                         resultBufferSize,
                         "Provisioning: Failed - Modem MQTT command is still busy. Retry provisioning after the current telemetry publish finishes. Modem=%s",
                         lastMqttModemResponse[0] != '\0' ? lastMqttModemResponse : "No modem response captured.");
            }
            return false;
        }
    }

    memcpy(openRemoteProvisioningRequest, requestJson, requestLength);
    openRemoteProvisioningRequest[requestLength] = '\0';
    openRemoteProvisioningActive = true;
    openRemoteProvisioningComplete = false;
    openRemoteProvisioningSuccess = false;
    openRemoteProvisioningRequestPublished = false;
    openRemoteProvisioningRealm[0] = '\0';
    openRemoteProvisioningAssetID[0] = '\0';
    ResetOpenRemoteProvisioningReceiveCapture();
    snprintf(openRemoteProvisioningStatus, sizeof(openRemoteProvisioningStatus), "OpenRemote provisioning request queued.");

    cellularTestBypassGpsWait = true;
    telemetryBatchActive = false;
    telemetryBatchStartedAt = 0;
    activeTelemetryAttributeIndex = -1;
    mqttConnected = false;
    mqttSubscribed = false;
    mqttClientState = MQTT_CLIENT_IDLE;
    nextMqttPublishAt = 0;
    mqttFailureMessage[0] = '\0';
    lastMqttModemResponse[0] = '\0';
    ClearQueuedSimCommand(MQTT_CONNECT);
    ClearQueuedSimCommand(MQTT_PUBLISH);
    ClearQueuedSimCommand(MQTT_SUBSCRIBE);
    ClearQueuedSimCommand(MQTT_UNSUBSCRIBE);

    uint32_t start = millis();
    if (!IsCellularInternetReady() && !simCommandPending)
    {
        StartCellularDataConnection(start);
    }

    while ((millis() - start) < OPENREMOTE_PROVISIONING_TIMEOUT_MS)
    {
        UpdateSIM7600();

        if (openRemoteProvisioningComplete)
        {
            break;
        }

        if (mqttClientState == MQTT_CLIENT_FAILED)
        {
            snprintf(openRemoteProvisioningStatus,
                     sizeof(openRemoteProvisioningStatus),
                     "%s",
                     mqttFailureMessage[0] != '\0' ? mqttFailureMessage : "MQTT provisioning publish failed.");
            openRemoteProvisioningComplete = true;
            openRemoteProvisioningSuccess = false;
            break;
        }

        if (cellularDataState == CELLULAR_DATA_FAILED)
        {
            snprintf(openRemoteProvisioningStatus,
                     sizeof(openRemoteProvisioningStatus),
                     "Packet data failed before provisioning: %s",
                     cellularDataFailureMessage[0] != '\0' ? cellularDataFailureMessage : "No packet data failure detail available.");
            openRemoteProvisioningComplete = true;
            openRemoteProvisioningSuccess = false;
            break;
        }

        if (cellularDataState == CELLULAR_DATA_CONNECTED && internetProbeComplete && !internetProbeSucceeded)
        {
            snprintf(openRemoteProvisioningStatus,
                     sizeof(openRemoteProvisioningStatus),
                     "Internet check failed before provisioning. Packet data is active, but DNS did not succeed.");
            openRemoteProvisioningComplete = true;
            openRemoteProvisioningSuccess = false;
            break;
        }

        IWatchdog.reload();
        delay(5);
    }

    if (!openRemoteProvisioningComplete)
    {
        if (openRemoteProvisioningRequestPublished)
        {
            snprintf(openRemoteProvisioningStatus,
                     sizeof(openRemoteProvisioningStatus),
                     "OpenRemote provisioning timed out waiting for response after request publish.");
        }
        else if (mqttSubscribed)
        {
            snprintf(openRemoteProvisioningStatus,
                     sizeof(openRemoteProvisioningStatus),
                     "OpenRemote provisioning timed out after subscribing to the response topic.");
        }
        else
        {
            snprintf(openRemoteProvisioningStatus,
                     sizeof(openRemoteProvisioningStatus),
                     "OpenRemote provisioning request timed out while waiting for cellular/MQTT readiness. Data=%s Internet=%s.",
                     GetCellularDataStateName(),
                     internetProbeComplete ? (internetProbeSucceeded ? "OK" : "Failed") : "Pending");
        }
        openRemoteProvisioningSuccess = false;
    }

    bool success = openRemoteProvisioningSuccess;
    if (resultBuffer != nullptr && resultBufferSize > 0)
    {
        snprintf(resultBuffer,
                 resultBufferSize,
                 "Provisioning: %s - %s Realm=%s AssetId=%s Host=%s:%u TLS=%s Auth=%s MqttId=%s Topic=provisioning/%s/request Modem=%s Response=%s",
                 success ? "OK" : "Failed",
                 openRemoteProvisioningStatus[0] != '\0' ? openRemoteProvisioningStatus : "No provisioning status available.",
                 openRemoteProvisioningRealm[0] != '\0' ? openRemoteProvisioningRealm : "unknown",
                 openRemoteProvisioningAssetID[0] != '\0' ? openRemoteProvisioningAssetID : "unknown",
                 CellularParams.OpenRemoteHost,
                 CellularParams.OpenRemotePort,
                 CellularParams.UseTLS ? "on" : "off",
                 lastMqttConnectAuthMode,
                 lastMqttConnectClientID[0] != '\0' ? lastMqttConnectClientID : "unknown",
                 CellularParams.ClientID,
                 lastMqttModemResponse[0] != '\0' ? lastMqttModemResponse : "No modem response captured.",
                 openRemoteProvisioningResponse[0] != '\0' ? openRemoteProvisioningResponse : "none");
    }

    openRemoteProvisioningActive = false;
    openRemoteProvisioningComplete = false;
    openRemoteProvisioningRequest[0] = '\0';
    cellularTestBypassGpsWait = false;
    mqttConnected = false;
    mqttSubscribed = false;
    mqttClientState = MQTT_CLIENT_IDLE;
    nextMqttPublishAt = 0;
    return success;
}

void GetCellularDiagnosticReport(char *buffer, size_t bufferSize)
{
    if (buffer == nullptr || bufferSize == 0)
    {
        return;
    }

    uint32_t now = millis();

    const char *settingsStatus = "OK";
    const char *settingsMessage = "Mobile data and APN are configured.";
    if (SystemParams.AllowData == 0)
    {
        settingsStatus = "Blocked";
        settingsMessage = "Mobile data is disabled in system settings.";
    }
    else if (CellularParams.APN[0] == '\0')
    {
        settingsStatus = "Blocked";
        settingsMessage = "APN is blank. Enter the APN supplied by the SIM provider.";
    }
    else if (IsOpenRemotePublishTopic() && CellularParams.MQTTUsername[0] != '\0' && !IsOpenRemoteMqttUsernameRealmPrefixed())
    {
        settingsStatus = "Warning";
        settingsMessage = "OpenRemote MQTT username should be realm:service-user.";
    }
    else if (!IsOpenRemotePublishTopicClientIDMatched())
    {
        settingsStatus = "Warning";
        settingsMessage = "OpenRemote MQTT topic device ID must match the MQTT client ID.";
    }

    const char *storageStatus = CellularCRCValid ? "OK" : "Invalid";
    const char *storageMessage = CellularCRCValid
                                     ? (CellularParams.APN[0] != '\0' ? "Cellular config loaded from EEPROM with APN present." : "Cellular config loaded from EEPROM but APN is blank.")
                                     : "Cellular EEPROM block CRC/version is invalid; defaults are active.";

    const char *gpsStatus = "Disabled";
    const char *gpsMessage = "GPS is disabled in system settings.";
    char gpsMessageBuffer[256];
    if (SystemParams.AllowGPS)
    {
        if (GPSFix)
        {
            gpsStatus = "Fix";
            snprintf(gpsMessageBuffer,
                     sizeof(gpsMessageBuffer),
                     "GNSS is enabled and has a current fix. %s Last=%s",
                     lastGPSParseStatus,
                     lastGNSSInfoResponse[0] != '\0' ? lastGNSSInfoResponse : "none");
            gpsMessage = gpsMessageBuffer;
        }
        else if (hasSeenGNSSResponse)
        {
            gpsStatus = "No fix";
            snprintf(gpsMessageBuffer,
                     sizeof(gpsMessageBuffer),
                     "GNSS is responding, but no usable fix has been accepted yet. %s Last=%s",
                     lastGPSParseStatus,
                     lastGNSSInfoResponse[0] != '\0' ? lastGNSSInfoResponse : "none");
            gpsMessage = gpsMessageBuffer;
        }
        else if (previousGPSEnable)
        {
            gpsStatus = "Waiting";
            gpsMessage = "GNSS is enabled; waiting for the first SIM7600 GNSS response.";
        }
        else
        {
            gpsStatus = "Starting";
            gpsMessage = "GPS is enabled in settings; waiting for the SIM7600 to accept GPS power-on.";
        }
    }

    const char *dataStatus = "Waiting";
    const char *dataMessage = "Waiting for SIM7600 to be ready.";
    char dataMessageBuffer[192];
    uint32_t activeCommandAgeSeconds = simCommandPending ? ((now - simCommandSentAt) / 1000UL) : 0;
    if (!IsCellularDataEnabled())
    {
        dataStatus = "Skipped";
        dataMessage = "Data connection test skipped until the settings above are fixed.";
    }
    else if (!CanStartCellularDataConnection())
    {
        dataStatus = "Waiting";
        dataMessage = "Waiting for the first GNSS response before sharing the SIM7600 UART with data commands.";
    }
    else if (dataConnected)
    {
        dataStatus = "Connected";
        dataMessage = "SIM7600 has an active PDP context and IP address.";
    }
    else
    {
        switch (cellularDataState)
        {
        case CELLULAR_DATA_SET_APN:
            dataStatus = "Connecting";
            snprintf(dataMessageBuffer, sizeof(dataMessageBuffer), "Setting the APN (%lus).", (unsigned long)activeCommandAgeSeconds);
            dataMessage = dataMessageBuffer;
            break;
        case CELLULAR_DATA_ATTACH:
            dataStatus = "Connecting";
            snprintf(dataMessageBuffer, sizeof(dataMessageBuffer), "Registering on the packet data network (%lus).", (unsigned long)activeCommandAgeSeconds);
            dataMessage = dataMessageBuffer;
            break;
        case CELLULAR_DATA_ACTIVATE_PDP:
            dataStatus = "Connecting";
            snprintf(dataMessageBuffer, sizeof(dataMessageBuffer), "Starting mobile data (%lus).", (unsigned long)activeCommandAgeSeconds);
            dataMessage = dataMessageBuffer;
            break;
        case CELLULAR_DATA_CHECK_ADDRESS:
            dataStatus = "Connecting";
            snprintf(dataMessageBuffer, sizeof(dataMessageBuffer), "Checking mobile data address (%lus).", (unsigned long)activeCommandAgeSeconds);
            dataMessage = dataMessageBuffer;
            break;
        case CELLULAR_DATA_OPEN_NETWORK:
            dataStatus = "Connected";
            dataMessage = "PDP context has an IP address; opening the SIM7600 IP stack.";
            break;
        case CELLULAR_DATA_SET_DNS:
            dataStatus = "Connected";
            dataMessage = "PDP context is active; configuring SIM7600 DNS servers.";
            break;
        case CELLULAR_DATA_PROBE_INTERNET:
            dataStatus = "Connected";
            dataMessage = "PDP context is active; checking internet reachability.";
            break;
        case CELLULAR_DATA_FAILED:
            dataStatus = "Failed";
            snprintf(dataMessageBuffer,
                     sizeof(dataMessageBuffer),
                     "%s Modem=%s.",
                     cellularDataFailureMessage[0] != '\0' ? cellularDataFailureMessage : "Packet data connection failed.",
                     lastCellularModemResponse[0] != '\0' ? lastCellularModemResponse : "No modem response captured");
            dataMessage = dataMessageBuffer;
            break;
        default:
            dataStatus = "Queued";
            dataMessage = "Data connection attempt has been requested.";
            break;
        }
    }

    char internetMessageBuffer[96];
    const char *internetStatus = "Waiting";
    const char *internetMessage = "Complete the packet data connection before testing internet reachability.";
    if (internetProbeComplete && internetProbeSucceeded)
    {
        internetStatus = "Connected";
        internetMessage = "DNS lookup succeeded over packet data.";
    }
    else if (internetProbeComplete)
    {
        internetStatus = "Failed";
        internetMessage = "Packet data is active, but DNS lookup failed.";
    }
    else if (cellularDataState == CELLULAR_DATA_PROBE_INTERNET)
    {
        internetStatus = "Testing";
        snprintf(internetMessageBuffer, sizeof(internetMessageBuffer), "Resolving %s through the SIM7600 data connection.", GetInternetProbeHost());
        internetMessage = internetMessageBuffer;
    }
    else if (cellularDataState == CELLULAR_DATA_OPEN_NETWORK)
    {
        internetStatus = "Testing";
        snprintf(internetMessageBuffer, sizeof(internetMessageBuffer), "Opening SIM7600 IP stack before resolving %s.", INTERNET_PROBE_HOST);
        internetMessage = internetMessageBuffer;
    }

    const bool mqttSettingsPresent = IsMqttConfigured();
    const char *mqttStatus = "Blocked";
    const char *mqttMessage = GetMissingMqttSettingMessage();
    char mqttMessageBuffer[256];
    char mqttPublishStatusBuffer[448];
    char locationStatusBuffer[192] = {0};
    char healthMessageBuffer[160];
    const char *healthStatus = "OK";
    const char *healthMessage = "Remote telemetry has no active fault.";

    uint32_t configuredPublishIntervalMs = GetConfiguredMqttPublishIntervalMs();
    uint32_t publishIntervalMs = GetMqttPublishIntervalMs();
    bool hasConfirmedTelemetry = lastMqttPublishSuccessAt != 0 && strcmp(lastMqttPublishAttribute, "OpenRemoteProvisioning") != 0;
    uint32_t lastSuccessAgeSeconds = hasConfirmedTelemetry ? ((now - lastMqttPublishSuccessAt) / 1000UL) : 0;
    uint32_t staleTelemetryThresholdMs = GetTelemetryOfflineTimeoutMs();
    uint8_t configuredTelemetryAttributes = CountConfiguredTelemetryAttributes();
    uint8_t batchProgress = CountTelemetryAttributesBeforeCursor();
    bool hasRecentTelemetry = hasConfirmedTelemetry && (now - lastMqttPublishSuccessAt) <= staleTelemetryThresholdMs;
    if (hasConfirmedTelemetry)
    {
        if (lastMqttPublishAttribute[0] != '\0')
        {
            snprintf(mqttPublishStatusBuffer,
                     sizeof(mqttPublishStatusBuffer),
                     "%u/%u selected telemetry values processed in this refresh. Refresh target %lus. Last update %lus ago: %s.",
                     (unsigned int)batchProgress,
                     (unsigned int)configuredTelemetryAttributes,
                     (unsigned long)(publishIntervalMs / 1000UL),
                     (unsigned long)lastSuccessAgeSeconds,
                     lastMqttPublishAttribute);
        }
        else
        {
            snprintf(mqttPublishStatusBuffer,
                     sizeof(mqttPublishStatusBuffer),
                     "%u/%u selected telemetry values processed in this refresh. Refresh target %lus. Last update %lus ago.",
                     (unsigned int)batchProgress,
                     (unsigned int)configuredTelemetryAttributes,
                     (unsigned long)(publishIntervalMs / 1000UL),
                     (unsigned long)lastSuccessAgeSeconds);
        }
    }
    else
    {
        snprintf(mqttPublishStatusBuffer,
                 sizeof(mqttPublishStatusBuffer),
                 "%u/%u selected telemetry values processed in this refresh. Refresh target %lus. No confirmed telemetry update yet.",
                 (unsigned int)batchProgress,
                 (unsigned int)configuredTelemetryAttributes,
                 (unsigned long)(publishIntervalMs / 1000UL));
    }

    if ((GetConfiguredTelemetryUploadMask() & TELEMETRY_UPLOAD_LOCATION) != 0)
    {
        if (lastLocationPublishSuccessAt != 0)
        {
            snprintf(locationStatusBuffer,
                     sizeof(locationStatusBuffer),
                     " Location update sent %lus ago.",
                     (unsigned long)((now - lastLocationPublishSuccessAt) / 1000UL));
        }
        else
        {
            snprintf(locationStatusBuffer, sizeof(locationStatusBuffer), " No confirmed location update yet.");
        }

        size_t currentLength = strlen(mqttPublishStatusBuffer);
        if (currentLength < sizeof(mqttPublishStatusBuffer) - 1)
        {
            snprintf(mqttPublishStatusBuffer + currentLength,
                     sizeof(mqttPublishStatusBuffer) - currentLength,
                     "%s",
                     locationStatusBuffer);
        }
    }

    if (mqttSettingsPresent && hasConfirmedTelemetry && !hasRecentTelemetry)
    {
        healthStatus = "Warning";
        snprintf(healthMessageBuffer,
                 sizeof(healthMessageBuffer),
                 "TEL-MQTT-010: no confirmed remote update for %lus.",
                 (unsigned long)lastSuccessAgeSeconds);
        healthMessage = healthMessageBuffer;
    }
    else if (mqttSettingsPresent && mqttConsecutiveFailureCount >= 10)
    {
        healthStatus = "Warning";
        snprintf(healthMessageBuffer,
                 sizeof(healthMessageBuffer),
                 "TEL-MQTT-010: remote telemetry has missed %lu consecutive updates.",
                 (unsigned long)mqttConsecutiveFailureCount);
        healthMessage = healthMessageBuffer;
    }

    if (mqttSettingsPresent && mqttConnected && mqttClientState == MQTT_CLIENT_CONNECTED)
    {
        mqttStatus = "Connected";
        if (CellularParams.SubscribeTopic[0] != '\0')
        {
            snprintf(mqttMessageBuffer,
                     sizeof(mqttMessageBuffer),
                     mqttSubscribed ? "Ready to publish selected telemetry. Subscription accepted %lus ago."
                                    : "Ready to publish selected telemetry. No subscription has been accepted yet.",
                     mqttSubscribed && lastMqttSubscribeSuccessAt != 0 ? (unsigned long)((now - lastMqttSubscribeSuccessAt) / 1000UL) : 0UL);
        }
        else
        {
            snprintf(mqttMessageBuffer,
                     sizeof(mqttMessageBuffer),
                     "Ready to publish selected telemetry.");
        }
        mqttMessage = mqttMessageBuffer;
    }
    else if (mqttSettingsPresent && mqttConnected && IsMqttSubscribeState())
    {
        mqttStatus = "Subscribing";
        snprintf(mqttMessageBuffer,
                 sizeof(mqttMessageBuffer),
                 "Requesting OpenRemote attribute subscription. Command age %lus.",
                 (unsigned long)activeCommandAgeSeconds);
        mqttMessage = mqttMessageBuffer;
    }
    else if (mqttSettingsPresent && mqttClientState == MQTT_CLIENT_FAILED)
    {
        if (hasConfirmedTelemetry)
        {
            mqttStatus = "Recovering";
            snprintf(mqttMessageBuffer,
                     sizeof(mqttMessageBuffer),
                     "Remote telemetry is retrying automatically.");
            mqttMessage = mqttMessageBuffer;
        }
        else
        {
            mqttStatus = "Failed";
            snprintf(mqttMessageBuffer,
                     sizeof(mqttMessageBuffer),
                     "%s Modem=%s Auth=%s MqttId=%s.",
                     mqttFailureMessage[0] != '\0' ? mqttFailureMessage : "Remote telemetry is retrying automatically.",
                     lastMqttModemResponse[0] != '\0' ? lastMqttModemResponse : "No modem response captured",
                     lastMqttConnectAuthMode,
                     lastMqttConnectClientID[0] != '\0' ? lastMqttConnectClientID : "unknown");
            mqttMessage = mqttMessageBuffer;
        }
    }
    else if (mqttSettingsPresent && dataConnected && internetProbeComplete && internetProbeSucceeded)
    {
        mqttStatus = "Connecting";
        mqttMessage = "Packet data is ready; connecting MQTT and publishing selected telemetry.";
    }
    else if (mqttSettingsPresent)
    {
        mqttStatus = "Waiting";
        mqttMessage = "MQTT settings look usable; waiting for packet data connection.";
    }

    snprintf(buffer, bufferSize,
             "SIM: %s - State=%s Pending=%s Active=%u GPSSeen=%u Queued=0x%04X.\nSettings: %s - %s\nStorage: %s - %s\nGPS: %s - %s\nData: %s - %s\nInternet: %s - %s\nMQTT: %s - %s\nTelemetry: %s - %s\nHealth: %s - %s",
             simCommandPending ? "Busy" : "Ready",
             GetSimStateName(),
             simCommandPending ? GetSimCommandName(pendingCommand) : "None",
             (unsigned int)activeSimCommandType,
             hasSeenGNSSResponse ? 1U : 0U,
             (unsigned int)queuedSimCommands,
             settingsStatus,
             settingsMessage,
             storageStatus,
             storageMessage,
             gpsStatus,
             gpsMessage,
             dataStatus,
             dataMessage,
             internetStatus,
             internetMessage,
             mqttStatus,
             mqttMessage,
             hasRecentTelemetry ? "OK" : (mqttPublishAttemptCount == 0 ? "Waiting" : "Warning"),
             mqttPublishStatusBuffer,
             healthStatus,
             healthMessage);
}

void UpdateSIM7600(SIM7600Commands command)
{
    QueueSimCommand(command);

    UpdateSIM7600();
}

void UpdateSIM7600()
{
    SIM7600Commands command = GPS;
    uint32_t now = millis();

    while (Serial1.available())
    {
        char incoming = Serial1.read();
        CaptureOpenRemoteProvisioningIncomingChar(incoming);
        if (simBufferLength < sizeof(simBuffer) - 1)
        {
            simBuffer[simBufferLength++] = incoming;
            simBuffer[simBufferLength] = '\0';
        }
        else
        {
            memmove(simBuffer, simBuffer + 1, sizeof(simBuffer) - 2);
            simBuffer[sizeof(simBuffer) - 2] = incoming;
            simBuffer[sizeof(simBuffer) - 1] = '\0';
            simBufferLength = sizeof(simBuffer) - 1;
        }
    }
#ifdef DEBUG
    Serial.print("Buffer: ");
    Serial.println(simBuffer);
#endif

    if (simBufferLength > 0)
    {
        ProcessSimResponseBuffer();
        CaptureOpenRemoteProvisioningResponseIfAvailable();
    }

    if (simCommandPending)
    {
        bool responseError = HasSimResponseError(simBuffer);
        bool responseSuccess = HasSimResponseSuccess(simBuffer);
        bool responseComplete = false;
        if (activeSimCommandType == SIM_ACTIVE_COMMAND_AT || activeSimCommandType == SIM_ACTIVE_COMMAND_GPS_POWER)
        {
            responseComplete = IsSimResponseComplete(simBuffer) || responseSuccess || responseError;
        }
        else if (pendingCommand == CELLULAR_CONNECT && cellularDataState == CELLULAR_DATA_PROBE_INTERNET)
        {
            responseComplete = HasInternetProbeResult(simBuffer) || responseError;
        }
        else if (pendingCommand == CELLULAR_CONNECT && cellularDataState == CELLULAR_DATA_OPEN_NETWORK)
        {
            responseComplete = HasNetworkOpenResult(simBuffer);
        }
        else if (pendingCommand == GPS)
        {
            responseComplete = HasCompleteGNSSInfoLine(simBuffer) || responseError;
        }
        else if (IsMqttCommand(pendingCommand))
        {
            responseComplete = IsMqttCommandResponseComplete(responseError);
        }
        else
        {
            responseComplete = IsSimResponseComplete(simBuffer);
        }
        bool commandTimedOut = (now - simCommandSentAt) >= GetActiveSimCommandTimeoutMs();
        if (!responseComplete && !commandTimedOut)
        {
            return;
        }

        if (activeSimCommandType == SIM_ACTIVE_COMMAND_AT)
        {
            if (commandTimedOut || responseError)
            {
                SIM7600State = 0;
            }
        }
        else if (activeSimCommandType == SIM_ACTIVE_COMMAND_GPS_POWER)
        {
            if (activeGPSEnableState)
            {
                // SIM7600 can return ERROR or no final result for CGPS=1 while CGNSSINFO still works.
                previousGPSEnable = true;
                gpsEnableAcceptedAt = now;
                nextGPSEnableRetryAt = now + SIM7600_GPS_RESPONSE_VERIFY_MS;
                QueueSimCommand(GPS);
            }
            else if (!commandTimedOut && responseSuccess)
            {
                previousGPSEnable = false;
                gpsEnableAcceptedAt = 0;
                nextGPSEnableRetryAt = 0;
            }
            else
            {
                previousGPSEnable = false;
                gpsEnableAcceptedAt = 0;
                nextGPSEnableRetryAt = SystemParams.AllowGPS ? (now + SIM7600_GPS_RESTART_DELAY_MS) : 0;
            }
            SIM7600State = 2;
        }
        else if (activeSimCommandType == SIM_ACTIVE_COMMAND_QUEUED)
        {
            if (IsCellularDataCommand(pendingCommand))
            {
                CaptureLastCellularModemResponse();
                ProcessCellularDataCommandResult(responseSuccess, responseError, commandTimedOut, now);
            }
            else if (IsMqttCommand(pendingCommand))
            {
                CaptureLastMqttModemResponse();
                if (pendingCommand == MQTT_CONNECT)
                {
                    ProcessMqttConnectCommandResult(responseError, commandTimedOut, now);
                }
                else if (pendingCommand == MQTT_PUBLISH)
                {
                    ProcessMqttPublishCommandResult(responseError, commandTimedOut, now);
                }
                else if (pendingCommand == MQTT_SUBSCRIBE)
                {
                    ProcessMqttSubscribeCommandResult(responseError, commandTimedOut, now);
                }
                else if (pendingCommand == MQTT_UNSUBSCRIBE)
                {
                    ProcessMqttUnsubscribeCommandResult(responseError, commandTimedOut, now);
                }
            }
        }

        simCommandPending = false;
        pendingCommand = GPS;
        activeSimCommandType = SIM_ACTIVE_COMMAND_NONE;
        activeGPSEnableState = false;
        ResetSimResponseBuffer();
    }

    switch (SIM7600State)
    {
    case 0:
        // Power up
        if ((int32_t)(now - simStartupDeadline) < 0)
        {
            break;
        }

        ResetSimResponseBuffer();
        Serial1.print("AT\r");
        activeSimCommandType = SIM_ACTIVE_COMMAND_AT;
        simCommandPending = true;
        simCommandSentAt = now;
        break;
    case 1:
        if ((int32_t)(now - simStartupDeadline) < 0)
        {
            break;
        }

        SendGPSPowerCommand(SystemParams.AllowGPS, now);
        break;
    case 2:
        // Ready for command
        if (previousGPSEnable != SystemParams.AllowGPS)
        {
            if (SystemParams.AllowGPS && nextGPSEnableRetryAt != 0 && (int32_t)(now - nextGPSEnableRetryAt) < 0)
            {
                // Keep data/network startup moving while GPS waits for its retry window.
            }
            else
            {
                SendGPSPowerCommand(SystemParams.AllowGPS, now);
                break;
            }
        }

        if (SystemParams.AllowGPS && previousGPSEnable && !hasSeenGNSSResponse &&
            gpsEnableAcceptedAt != 0 && nextGPSEnableRetryAt != 0 && (int32_t)(now - nextGPSEnableRetryAt) >= 0)
        {
            nextGPSEnableRetryAt = now + SIM7600_GPS_RESPONSE_VERIFY_MS;
            QueueSimCommand(GPS);
            break;
        }

        if (!IsCellularDataEnabled())
        {
            dataConnected = false;
            cellularTestBypassGpsWait = false;
            cellularDataState = CELLULAR_DATA_IDLE;
            mqttConnected = false;
            mqttClientState = MQTT_CLIENT_IDLE;
        }

        if (IsCellularDataEnabled() && !dataConnected &&
            (cellularDataState == CELLULAR_DATA_IDLE || cellularDataState == CELLULAR_DATA_FAILED) &&
            (int32_t)(now - nextCellularDataAttemptAt) >= 0)
        {
            StartCellularDataConnection(now);
        }

        if (IsCellularDataEnabled() && dataConnected && cellularDataState == CELLULAR_DATA_CONNECTED &&
            internetProbeComplete && !internetProbeSucceeded && !simCommandPending &&
            (int32_t)(now - nextCellularDataAttemptAt) >= 0)
        {
            internetProbeComplete = false;
            cellularDataState = CELLULAR_DATA_OPEN_NETWORK;
            QueueSimCommand(CELLULAR_CONNECT);
        }

        if (IsMqttConfigured() && dataConnected && internetProbeComplete && internetProbeSucceeded && !simCommandPending)
        {
            if (!mqttConnected && (mqttClientState == MQTT_CLIENT_IDLE || mqttClientState == MQTT_CLIENT_FAILED) && (int32_t)(now - nextMqttPublishAt) >= 0)
            {
                StartMqttConnection();
            }
            else if (mqttConnected && mqttClientState == MQTT_CLIENT_CONNECTED && (int32_t)(now - nextMqttPublishAt) >= 0)
            {
                StartMqttPublish(now);
            }
        }

        if (!TryDequeueNextSimCommand(&command))
        {
            break;
        }

        if (command == GPS && !SystemParams.AllowGPS)
        {
            break;
        }

        if (command != GPS && !IsCellularDataCommand(command) && !IsMqttCommand(command) && !cellularTestBypassGpsWait && !CanDispatchNonGPSCommandNow(millis()))
        {
            QueueSimCommand(command);
            break;
        }

        ResetSimResponseBuffer();
        pendingCommand = command;
        activeSimCommandType = SIM_ACTIVE_COMMAND_QUEUED;
        SendSimCommand(command);
        simCommandPending = true;
        simCommandSentAt = now;
        break;
    }
}

void parseGPSData(const char *response)
{
    uint32_t now = millis();

    if (response == nullptr)
    {
        return;
    }

    const char *payload = strstr(response, SIM7600_GNSSINFO_RESPONSE_TOKEN);
    if (payload == nullptr)
    {
        return;
    }

    hasSeenGNSSResponse = true;

    response = payload + strlen(SIM7600_GNSSINFO_RESPONSE_TOKEN);
    while (*response == ' ' || *response == '\t')
    {
        response++;
    }

    snprintf(lastGNSSInfoResponse, sizeof(lastGNSSInfoResponse), "%s", response);
    for (size_t i = 0; lastGNSSInfoResponse[i] != '\0'; i++)
    {
        if (lastGNSSInfoResponse[i] == '\r' || lastGNSSInfoResponse[i] == '\n')
        {
            lastGNSSInfoResponse[i] = ' ';
        }
    }

    char buffer[100]; // Temporary buffer to modify the input string
    strncpy(buffer, response, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0'; // Ensure null termination

    char *tokens[16] = {nullptr};
    int index = SplitCsvPreserveEmpty(buffer, tokens, 16);

    if (index < 16) // Ensure enough tokens are parsed
    {
#ifdef DEBUG
        Serial.println("Incomplete GPS data");
#endif
        snprintf(lastGPSParseStatus, sizeof(lastGPSParseStatus), "Incomplete CGNSSINFO fields (%d).", index);
        UpdateGPSFixState(false, now);
        return;
    }

    // Parse values
    int fixStatus = atoi(tokens[0]);

#ifdef DEBUG
    Serial.print("Fix status: ");
    Serial.println(fixStatus);
#endif

    if (fixStatus < 2 || tokens[4][0] == '\0' || tokens[5][0] == '\0' || tokens[6][0] == '\0' || tokens[7][0] == '\0')
    {
        snprintf(lastGPSParseStatus, sizeof(lastGPSParseStatus), "No valid GNSS fix fields (status %d).", fixStatus);
        UpdateGPSFixState(false, now);
        return;
    }

    // A valid SIM7600 GNSS fix should drive the user-facing GPS status immediately.
    // The plausibility filter below only decides whether to accept/update logged coordinates.
    lastGNSSFixSeenAt = now;
    GPSFix = true;
    snprintf(lastGPSParseStatus, sizeof(lastGPSParseStatus), "Valid CGNSSINFO fix received.");

    // Convert DMM to Decimal Degrees
    auto convertToDecimalDegrees = [](const char *degMin, const char *dir) -> float
    {
        float value = atof(degMin);
        int deg = int(value / 100);          // Extract degrees
        float minutes = value - (deg * 100); // Extract minutes
        float decimalDegrees = deg + (minutes / 60.0);

        if (dir[0] == 'S' || dir[0] == 'W')
        {
            decimalDegrees *= -1;
        }

        return decimalDegrees;
    };

    ParsedGPSFix candidate = {};
    candidate.latitude = convertToDecimalDegrees(tokens[4], tokens[5]);
    candidate.longitude = convertToDecimalDegrees(tokens[6], tokens[7]);
    candidate.altitude = atof(tokens[10]);
    candidate.speedKnots = atof(tokens[11]);

    int parsedYear = 0;
    if (strlen(tokens[8]) == 6)
    {
        sscanf(tokens[8], "%2d%2d%2d", &candidate.parsedDay, &candidate.parsedMonth, &parsedYear);
        candidate.parsedYear = parsedYear + 2000;
    }
    else
    {
        candidate.parsedDay = day;
        candidate.parsedMonth = month;
        candidate.parsedYear = year;
    }

    if (strlen(tokens[9]) >= 6)
    {
        sscanf(tokens[9], "%2d%2d%2d", &candidate.parsedHour, &candidate.parsedMinute, &candidate.parsedSecond);
    }
    else
    {
        candidate.parsedHour = hour;
        candidate.parsedMinute = minute;
        candidate.parsedSecond = second;
    }

    if (!IsCandidateFixPlausible(candidate, now))
    {
#ifdef DEBUG
        Serial.println("Rejected implausible GPS fix");
#endif
        snprintf(lastGPSParseStatus, sizeof(lastGPSParseStatus), "Valid fix seen; coordinates rejected by plausibility filter.");
        return;
    }

    AcceptCandidateFix(candidate, now);
}

uint8_t csq_to_bars()
{
    if (rssi < 0 || rssi > SIM7600_CSQ_RSSI_MAX)
    {
        return 0;
    }

    if (rssi <= 3)
    {
        return 0;
    }
    if (rssi <= 7)
    {
        return 1;
    }
    if (rssi <= 11)
    {
        return 2;
    }
    if (rssi <= 15)
    {
        return 3;
    }
    if (rssi <= 20)
    {
        return 4;
    }
    return 5;
}
