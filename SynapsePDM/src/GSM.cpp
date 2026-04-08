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

#include <ctype.h>
#include <math.h>
#include <stdio.h>

// #define DEBUG

uint32_t lastGPSTime = 0;

static bool hasAcceptedGPSFix = false;
static bool hasSeenGNSSResponse = false;
static float lastAcceptedLat = 0.0f;
static float lastAcceptedLon = 0.0f;
static float lastAcceptedSpeedKnots = 0.0f;

static bool previousGPSEnable = false;
static int rssi = SIM7600_CSQ_RSSI_UNKNOWN;
static uint8_t SIM7600State = 0; // 0 = Power up, 1 = Initialising, 2 = Ready for command, 4 = Wait response
static uint32_t simStartupDeadline = 0;
static uint32_t nextGPSEnableRetryAt = 0;
static char simBuffer[512];
static size_t simBufferLength = 0;
static SIM7600Commands pendingCommand = GPS;
static bool simCommandPending = false;
static uint32_t simCommandSentAt = 0;
static uint16_t queuedSimCommands = 0;
static uint8_t activeSimCommandType = 0;
static bool activeGPSEnableState = false;

float lat, lon, speed, alt, accuracy;
int vsat, usat, year, month, day, hour, minute, second;

bool GPSFix = false;

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

static uint16_t CommandBit(SIM7600Commands command)
{
    return static_cast<uint16_t>(1U << static_cast<uint8_t>(command));
}

static bool ShouldQueueSimCommand(SIM7600Commands command)
{
    if (command == GPS)
    {
        return true;
    }
    
    if (command == MODULE_TEMPERATURE)
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

    const SIM7600Commands commandOrder[] = {
        GPS,
        MODULE_TEMPERATURE,
        SIGNAL_QUALITY,
        NETWORK_MODE,
        HTTP,
        SMS,
        MQTT,
        MQTT_PUBLISH,
        MQTT_SUBSCRIBE,
        MQTT_UNSUBSCRIBE,
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

static void SendGPSPowerCommand(bool enableGPS, uint32_t now)
{
    ResetSimResponseBuffer();
    if (enableGPS)
    {
        Serial1.print("AT+CGPS=1\r");
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
    case MQTT_PUBLISH:
        break;
    case MQTT_SUBSCRIBE:
        break;
    case MQTT_UNSUBSCRIBE:
        break;
    case MQTT_CONNECT:
        break;
    case MQTT_DISCONNECT:
        break;
    case MQTT_PING:
        break;
    case MQTT_STATUS:
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

void InitialiseGSM(bool enableData)
{
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
    SIM7600State = 0;
    simCommandPending = false;
    pendingCommand = GPS;
    simCommandSentAt = 0;
    queuedSimCommands = 0;
    activeSimCommandType = SIM_ACTIVE_COMMAND_NONE;
    activeGPSEnableState = false;
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
        if (simBufferLength < sizeof(simBuffer) - 1)
        {
            simBuffer[simBufferLength++] = Serial1.read();
            simBuffer[simBufferLength] = '\0';
        }
        else
        {
            Serial1.read();
        }
    }
#ifdef DEBUG
    Serial.print("Buffer: ");
    Serial.println(simBuffer);
#endif

    if (simBufferLength > 0)
    {
        ProcessSimResponseBuffer();
    }

    if (simCommandPending)
    {
        bool responseComplete = IsSimResponseComplete(simBuffer);
        bool responseSuccess = HasSimResponseSuccess(simBuffer);
        bool responseError = HasSimResponseError(simBuffer);
        bool commandTimedOut = (now - simCommandSentAt) >= SIM7600_RESPONSE_TIMEOUT_MS;
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
            if (!commandTimedOut && responseSuccess)
            {
                previousGPSEnable = activeGPSEnableState;
                nextGPSEnableRetryAt = 0;
            }
            else
            {
                previousGPSEnable = false;
                nextGPSEnableRetryAt = now + SIM7600_GPS_ENABLE_RETRY_MS;
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
                break;
            }

            SendGPSPowerCommand(SystemParams.AllowGPS, now);
            break;
        }

        if (!TryDequeueNextSimCommand(&command))
        {
            break;
        }

        if (command == GPS && !SystemParams.AllowGPS)
        {
            break;
        }

        if (command != GPS && !CanDispatchNonGPSCommandNow(millis()))
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

    char buffer[100]; // Temporary buffer to modify the input string
    strncpy(buffer, response, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0'; // Ensure null termination

    char *tokens[15] = {nullptr};
    char *token = strtok(buffer, ",");
    int index = 0;

    while (token != nullptr && index < 15)
    {
        tokens[index++] = token;
        token = strtok(nullptr, ",");
    }

    if (index < 13) // Ensure enough tokens are parsed
    {
#ifdef DEBUG
        Serial.println("Incomplete GPS data");
#endif
        return;
    }

    // Parse values
    int fixStatus = atoi(tokens[0]);

#ifdef DEBUG
    Serial.print("Fix status: ");
    Serial.println(fixStatus);
#endif

    if (fixStatus == 0)
    {
        UpdateGPSFixState(false, now);
        return;
    }

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
        UpdateGPSFixState(false, now);
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
