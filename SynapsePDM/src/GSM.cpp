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

//#define DEBUG
#define BAUD_RATE 115200

uint32_t lastGPSTime = 0;

bool moduleReady = false; // Track module initialization

float lat, lon, speed, alt, accuracy;
int vsat, usat, year, month, day, hour, minute, second;

bool GPSFix = false;

bool previousGPSEnable = false;

uint8_t SIM7600State = 0; // 0 = Power up, 1 = Initialising, 2 = Ready for command, 4 = Wait response

char simBuffer[512];

void InitialiseGSM(bool enableData)
{
    pinMode(SIM_PWR, OUTPUT);
    pinMode(SIM_RST, OUTPUT);
    pinMode(SIM_FLIGHT, OUTPUT);

    digitalWrite(SIM_RST, LOW);
    digitalWrite(SIM_FLIGHT, LOW);

    // Power the module on
    digitalWrite(SIM_PWR, HIGH);
    Serial1.begin(BAUD_RATE);
    previousGPSEnable = SystemParams.AllowGPS;
    SIM7600State = 0;
#ifdef DEBUG
    Serial.println("Initializing SIM7600G...");
#endif
}

void UpdateSIM7600(SIM7600Commands command)
{
    // Clear the buffer before reading
    memset(simBuffer, 0, sizeof(simBuffer));
    size_t bytesRead = 0;

    unsigned long startTime = millis();

    while (Serial1.available())
    {
        if (bytesRead < sizeof(simBuffer) - 1)
        {
            simBuffer[bytesRead++] = Serial1.read();
        }
    }
#ifdef DEBUG
    Serial.print("GPS buffer: ");
    Serial.println(simBuffer);
    Serial.print("Read GPS bytes time: ");
    Serial.println(millis() - startTime);
#endif

    switch (SIM7600State)
    {
    case 0:
        // Power up
        Serial1.print("AT\r");
        break;
    case 1:
        if (SystemParams.AllowGPS)
        {
            Serial1.print("AT+CGPS=1\r");
        }
        else
        {
            Serial1.print("AT+CGPS=0\r");
        }
        break;
    case 2:
        // Ready for command
        switch (command)
        {
        case GPS:
            if (SystemParams.AllowGPS)
            {
                // GPS was enabled during runtime, make sure it's on
                if (previousGPSEnable != SystemParams.AllowGPS)
                {
                    Serial1.print("AT+CGPS=1\r");
                    previousGPSEnable = SystemParams.AllowGPS;
                }
                Serial1.print("AT+CGNSSINFO\r");
#ifdef DEBUG
                Serial.println("Requesting GPS info...");
#endif
            }
            else
            {
                // GPS was disabled during runtime, ensure it's off
                if (previousGPSEnable != SystemParams.AllowGPS)
                {
                    Serial1.print("AT+CGPS=0\r");
                    previousGPSEnable = SystemParams.AllowGPS;
                    GPSFix = false;
                }
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
        }
        break;
    }

    // Check response
    if (strstr(simBuffer, "AT") != nullptr || strstr(simBuffer, "OK") != nullptr || strstr(simBuffer, "DONE") != nullptr || strstr(simBuffer, "READY") != nullptr)
    {
        if (SIM7600State == 0)
        {
            SIM7600State = 1; // Transition to initialise GPS state
        }
        else if (SIM7600State == 1)
        {
            SIM7600State = 2; // Transition to enable GPS
        }
    }

    if (strstr(simBuffer, "ERROR") != nullptr)
    {
        SIM7600State = 2; // Transition to Ready for command state
    }

    if (strstr(simBuffer, "+CGNSSINFO") != nullptr)
    {
        parseGPSData(simBuffer); // Parse GPS data
        SIM7600State = 2;        // Transition to Ready for command state
    }
}

void parseGPSData(const char *response)
{
    response += 27; // Skip "+CGNSSINFO: "

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
        GPSFix = false; // No fix
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
        GPSFix = false; // No fix
    }
    else
    {
        GPSFix = true; // Valid fix
    }

    // Convert DMM to Decimal Degrees
    auto convertToDecimalDegrees = [](const char *degMin, const char *dir) -> float
    {
        float value = atof(degMin);
        int deg = int(value / 100);          // Extract degrees
        float minutes = value - (deg * 100); // Extract minutes
        float decimalDegrees = deg + (minutes / 60.0);

        // Apply hemisphere correction
        if (dir[0] == 'S' || dir[0] == 'W')
            decimalDegrees *= -1;

        return decimalDegrees;
    };

    lat = convertToDecimalDegrees(tokens[4], tokens[5]); // Latitude
    lon = convertToDecimalDegrees(tokens[6], tokens[7]); // Longitude
    alt = atof(tokens[10]);
    speed = atof(tokens[11]);
    float course = atof(tokens[12]);

    // Extract date (DDMMYY)
    if (strlen(tokens[8]) == 6)
        sscanf(tokens[8], "%2d%2d%2d", &day, &month, &year);
    year += 2000; // Convert two-digit year to full year (e.g., 24 -> 2024)

    // Extract time (HHMMSS.s)
    if (strlen(tokens[9]) >= 6)
        sscanf(tokens[9], "%2d%2d%2d", &hour, &minute, &second);
}
