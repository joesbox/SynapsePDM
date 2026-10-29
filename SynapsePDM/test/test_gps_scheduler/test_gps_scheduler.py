"""Compile production GPS freshness, modem timeout and URC routing helpers with native state."""

from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


def function(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


HARNESS = r"""
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#define CHECK(c) do { if (!(c)) { std::fprintf(stderr, "%d: %s\n", __LINE__, #c); std::exit(1); } } while (0)
constexpr uint32_t GPS_FIX_GRACE_PERIOD_MS = 6000UL;
constexpr uint32_t SIM7600_RESPONSE_TIMEOUT_MS = 2000UL;
constexpr uint32_t SIM7600_ASYNC_ACCEPT_TIMEOUT_MS = 5000UL;
constexpr uint32_t SIM7600_RESYNC_TIMEOUT_MS = 1000UL;
constexpr uint32_t SIM7600_DATA_RESPONSE_TIMEOUT_MS = 30000UL;
constexpr uint32_t SIM7600_DNS_RESPONSE_TIMEOUT_MS = 12000UL;
constexpr uint32_t SIM7600_MQTT_CONNECT_RESPONSE_TIMEOUT_MS = 90000UL;
constexpr uint32_t SIM7600_MQTT_PROMPT_TIMEOUT_MS = 5000UL;
constexpr uint32_t SIM7600_MQTT_PUBLISH_RESPONSE_TIMEOUT_MS = 10000UL;
enum SIM7600Commands { GPS, MQTT_CONNECT, MQTT_PUBLISH, MQTT_SUBSCRIBE, MQTT_UNSUBSCRIBE, CELLULAR_CONNECT, CELLULAR_DISCONNECT };
enum { SIM_ACTIVE_COMMAND_QUEUED = 3, SIM_ACTIVE_COMMAND_RESYNC = 4 };
enum {
    MQTT_CLIENT_CONNECT = 1, MQTT_CLIENT_SET_TOPIC, MQTT_CLIENT_SET_PAYLOAD,
    MQTT_CLIENT_WRITE_TOPIC, MQTT_CLIENT_WRITE_PAYLOAD, MQTT_CLIENT_PUBLISH,
    MQTT_CLIENT_SET_SUBSCRIBE_TOPIC, MQTT_CLIENT_SET_UNSUBSCRIBE_TOPIC,
    MQTT_CLIENT_CLEAN_DISCONNECT, MQTT_CLIENT_CLEAN_RELEASE, MQTT_CLIENT_CLEAN_STOP,
    MQTT_CLIENT_START, MQTT_CLIENT_ACQUIRE, MQTT_CLIENT_WRITE_SUBSCRIBE_TOPIC,
    MQTT_CLIENT_WRITE_UNSUBSCRIBE_TOPIC
};
enum { CELLULAR_DATA_OPEN_NETWORK = 1, CELLULAR_DATA_PROBE_INTERNET, CELLULAR_DATA_CHECK_REGISTRATION };
int activeSimCommandType = SIM_ACTIVE_COMMAND_QUEUED;
SIM7600Commands pendingCommand = GPS;
int mqttClientState = 0;
int cellularDataState = 0;
bool hasAcceptedGPSFix = false;
bool mqttConnected = true;
uint32_t queuedSimCommands = 0;
static uint32_t CommandBit(SIM7600Commands command) { return 1UL << command; }
bool hasSeenGNSSFix = false;
bool GPSFix = false;
uint32_t lastGPSTime = 0;
uint32_t lastGNSSFixSeenAt = 0;
float lat = 0, lon = 0, speed = 0, alt = 0, accuracy = 0;
int vsat = 0, usat = 0;
static void ClearLiveGPSData() {
    lat = lon = speed = alt = accuracy = 0;
    vsat = usat = 0;
}
static bool IsMqttCommand(SIM7600Commands command) {
    return command == MQTT_CONNECT || command == MQTT_PUBLISH ||
           command == MQTT_SUBSCRIBE || command == MQTT_UNSUBSCRIBE;
}
"""

CASES = r"""
int main(int argc, char **argv) {
    CHECK(argc == 2);
    const char *test = argv[1];
    if (!std::strcmp(test, "timeout-profile")) {
        pendingCommand = CELLULAR_CONNECT;
        cellularDataState = 0;
        CHECK(GetActiveSimCommandTimeoutMs() == 30000);
        cellularDataState = CELLULAR_DATA_OPEN_NETWORK;
        CHECK(GetActiveSimCommandTimeoutMs() == 12000);
        cellularDataState = CELLULAR_DATA_PROBE_INTERNET;
        CHECK(GetActiveSimCommandTimeoutMs() == 12000);
        cellularDataState = CELLULAR_DATA_CHECK_REGISTRATION;
        CHECK(GetActiveSimCommandTimeoutMs() == 2000);

        pendingCommand = MQTT_CONNECT;
        mqttClientState = MQTT_CLIENT_CONNECT;
        CHECK(GetActiveSimCommandTimeoutMs() == 90000);
        mqttClientState = 0;
        CHECK(GetActiveSimCommandTimeoutMs() == 30000);

        pendingCommand = MQTT_PUBLISH;
        mqttClientState = MQTT_CLIENT_SET_TOPIC;
        CHECK(GetActiveSimCommandTimeoutMs() == 5000);
        mqttClientState = MQTT_CLIENT_PUBLISH;
        CHECK(GetActiveSimCommandTimeoutMs() == 10000);

        pendingCommand = MQTT_SUBSCRIBE;
        mqttClientState = MQTT_CLIENT_SET_SUBSCRIBE_TOPIC;
        CHECK(GetActiveSimCommandTimeoutMs() == 5000);
        pendingCommand = MQTT_UNSUBSCRIBE;
        mqttClientState = MQTT_CLIENT_SET_UNSUBSCRIBE_TOPIC;
        CHECK(GetActiveSimCommandTimeoutMs() == 5000);

        pendingCommand = GPS;
        CHECK(GetActiveSimCommandTimeoutMs() == 2000);
    } else if (!std::strcmp(test, "phase-timeout")) {
        pendingCommand = MQTT_CONNECT;
        mqttClientState = MQTT_CLIENT_CONNECT;
        activeSimAsyncSlot = SIM_URC_MQTT_CONNECT;
        CHECK(GetActiveSimCommandPhaseTimeoutMs() == 5000);
        activeSimAsyncSlot = SIM_URC_NONE;
        CHECK(GetActiveSimCommandPhaseTimeoutMs() == 90000);
        pendingCommand = GPS;
        CHECK(GetActiveSimCommandPhaseTimeoutMs() == 2000);
        activeSimCommandType = SIM_ACTIVE_COMMAND_RESYNC;
        CHECK(GetActiveSimCommandPhaseTimeoutMs() == 1000);
    } else if (!std::strcmp(test, "async-slot-map")) {
        pendingCommand = MQTT_CONNECT;
        mqttClientState = MQTT_CLIENT_CLEAN_DISCONNECT;
        CHECK(GetAsyncUrcSlotForCommand(MQTT_CONNECT) == SIM_URC_MQTT_DISC);
        mqttClientState = MQTT_CLIENT_CLEAN_RELEASE;
        CHECK(GetAsyncUrcSlotForCommand(MQTT_CONNECT) == SIM_URC_NONE);
        mqttClientState = MQTT_CLIENT_START;
        CHECK(GetAsyncUrcSlotForCommand(MQTT_CONNECT) == SIM_URC_MQTT_START);
        mqttClientState = MQTT_CLIENT_ACQUIRE;
        CHECK(GetAsyncUrcSlotForCommand(MQTT_CONNECT) == SIM_URC_NONE);
        mqttClientState = MQTT_CLIENT_CONNECT;
        CHECK(GetAsyncUrcSlotForCommand(MQTT_CONNECT) == SIM_URC_MQTT_CONNECT);
        mqttClientState = MQTT_CLIENT_SET_TOPIC;
        CHECK(GetAsyncUrcSlotForCommand(MQTT_PUBLISH) == SIM_URC_NONE);
        mqttClientState = MQTT_CLIENT_PUBLISH;
        CHECK(GetAsyncUrcSlotForCommand(MQTT_PUBLISH) == SIM_URC_MQTT_PUB);
        mqttClientState = MQTT_CLIENT_WRITE_SUBSCRIBE_TOPIC;
        CHECK(GetAsyncUrcSlotForCommand(MQTT_SUBSCRIBE) == SIM_URC_MQTT_SUB);
        cellularDataState = CELLULAR_DATA_OPEN_NETWORK;
        CHECK(GetAsyncUrcSlotForCommand(CELLULAR_CONNECT) == SIM_URC_NETOPEN);
        cellularDataState = CELLULAR_DATA_PROBE_INTERNET;
        CHECK(GetAsyncUrcSlotForCommand(CELLULAR_CONNECT) == SIM_URC_CDNSGIP);
        CHECK(GetAsyncUrcSlotForCommand(GPS) == SIM_URC_NONE);
    } else if (!std::strcmp(test, "urc-router")) {
        // URCs interleaved with a GPS reply are captured by slot, whichever command owns the UART.
        const char *stream =
            "AT+CGNSSINFO\r\r\n+CGNSSINFO: 2,09,,,\r\n\r\n+CMQTTCONNECT: 0,31\r\nOK\r\n"
            ">+CMQTTPUB: 0,0\r\n+CMQTTCONNLOST: 0,3\r\n";
        for (const char *c = stream; *c; c++) {
            ProcessSimLineChar(*c);
        }
        CHECK(simUrcSeen[SIM_URC_MQTT_CONNECT]);
        CHECK(!std::strcmp(simUrcLines[SIM_URC_MQTT_CONNECT], "+CMQTTCONNECT: 0,31"));
        CHECK(simUrcSeen[SIM_URC_MQTT_PUB]);
        CHECK(!std::strcmp(simUrcLines[SIM_URC_MQTT_PUB], "+CMQTTPUB: 0,0"));
        CHECK(!simUrcSeen[SIM_URC_MQTT_START]);
        CHECK(simUrcSeen[SIM_URC_MQTT_CONNLOST]);
        CHECK(!mqttConnected && mqttConnectionLostCause == 3);
        ClearSimUrc(SIM_URC_MQTT_CONNECT);
        CHECK(!simUrcSeen[SIM_URC_MQTT_CONNECT] && simUrcLines[SIM_URC_MQTT_CONNECT][0] == 0);
        ProcessSimLineChar('+');
        CHECK(!simUrcSeen[SIM_URC_MQTT_CONNECT]);
    } else if (!std::strcmp(test, "registration-parse")) {
        const char *searching = "AT+CGREG?;+CEREG?\r\r\n+CGREG: 0,2\r\n\r\n+CEREG: 0,4\r\n\r\nOK\r\n";
        CHECK(GetRegistrationStatus(searching, "+CGREG:") == 2);
        CHECK(GetRegistrationStatus(searching, "+CEREG:") == 4);
        CHECK(!IsRegistrationStatusRegistered(2) && !IsRegistrationStatusRegistered(4));
        const char *roaming = "\r\n+CGREG: 0,1\r\n\r\n+CEREG: 2,5,\"1A2B\",\"01A2B3C4\",7\r\n\r\nOK\r\n";
        CHECK(GetRegistrationStatus(roaming, "+CEREG:") == 5);
        CHECK(IsRegistrationStatusRegistered(1) && IsRegistrationStatusRegistered(5));
        CHECK(GetRegistrationStatus("\r\nERROR\r\n", "+CEREG:") == -1);
    } else if (!std::strcmp(test, "prompt-input-priority")) {
        SIM7600Commands command = GPS;
        mqttClientState = MQTT_CLIENT_WRITE_TOPIC;
        CHECK(!IsSimPromptInputQueued(&command));
        queuedSimCommands = CommandBit(MQTT_PUBLISH) | CommandBit(GPS);
        CHECK(IsSimPromptInputQueued(&command) && command == MQTT_PUBLISH);
        queuedSimCommands = CommandBit(MQTT_SUBSCRIBE) | CommandBit(GPS);
        mqttClientState = MQTT_CLIENT_WRITE_SUBSCRIBE_TOPIC;
        CHECK(IsSimPromptInputQueued(&command) && command == MQTT_SUBSCRIBE);
        queuedSimCommands = CommandBit(MQTT_UNSUBSCRIBE);
        mqttClientState = MQTT_CLIENT_WRITE_UNSUBSCRIBE_TOPIC;
        CHECK(IsSimPromptInputQueued(&command) && command == MQTT_UNSUBSCRIBE);
        mqttClientState = MQTT_CLIENT_PUBLISH;
        queuedSimCommands = CommandBit(MQTT_PUBLISH);
        CHECK(!IsSimPromptInputQueued(&command));
    } else if (!std::strcmp(test, "accepted-fix-expiry")) {
        hasAcceptedGPSFix = true;
        lastGPSTime = 1000;
        lat = 51.5f;
        lon = -0.1f;
        UpdateGPSFixState(false, 6999);
        CHECK(GPSFix && lat == 51.5f);
        UpdateGPSFixState(false, 7001);
        CHECK(!GPSFix && lat == 0 && lon == 0);
    } else if (!std::strcmp(test, "gnss-fix-expiry")) {
        hasSeenGNSSFix = true;
        lastGNSSFixSeenAt = 1000;
        lat = 51.5f;
        UpdateGPSFixState(false, 6999);
        CHECK(GPSFix && lat == 51.5f);
        UpdateGPSFixState(false, 7001);
        CHECK(!GPSFix && lat == 0);
    } else if (!std::strcmp(test, "millis-wrap")) {
        hasSeenGNSSFix = true;
        lastGNSSFixSeenAt = 0xFFFFFF00UL;
        UpdateGPSFixState(false, 0x00000500UL);
        CHECK(GPSFix);
        UpdateGPSFixState(false, 0x00001800UL);
        CHECK(!GPSFix);
    } else {
        CHECK(false);
    }
}
"""


class GpsSchedulerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (ROOT / "src" / "GSM.cpp").read_text(encoding="utf-8")
        signatures = [
            "static uint32_t GetActiveSimCommandTimeoutMs()",
            "static void UpdateGPSFixState(",
        ]
        state_start = source.index("enum SimUrcSlot")
        state_end = source.index("static int mqttConnectionLostCause = -1;")
        urc_state = source[state_start:state_end] + "static int mqttConnectionLostCause = -1;\n"
        signatures += [
            "static uint32_t GetActiveSimCommandPhaseTimeoutMs()",
            "static void ClearSimUrc(",
            "static void ProcessSimLine(",
            "static void ProcessSimLineChar(",
            "static uint8_t GetAsyncUrcSlotForCommand(",
            "static bool IsSimPromptInputQueued(",
            "static int GetRegistrationStatus(",
            "static bool IsRegistrationStatusRegistered(",
        ]
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        path = Path(cls.temp.name)
        cpp = path / "gps_scheduler.cpp"
        cpp.write_text(
            HARNESS + urc_state + "\n".join(function(source, item) for item in signatures) + CASES,
            encoding="utf-8",
        )
        cls.executable = path / "gps_scheduler.exe"
        subprocess.run(
            [sys.executable, "-m", "ziglang", "c++", "-std=c++17", "-O2",
             str(cpp), "-o", str(cls.executable)],
            check=True,
        )

    def test_modem_commands_keep_their_configured_timeouts(self):
        subprocess.run([str(self.executable), "timeout-profile"], check=True)

    def test_async_modem_results_release_the_uart(self):
        for case in ("phase-timeout", "async-slot-map", "urc-router", "prompt-input-priority", "registration-parse"):
            with self.subTest(case=case):
                subprocess.run([str(self.executable), case], check=True)

    def test_gps_fix_expires_and_clears_cached_coordinates(self):
        for case in ("accepted-fix-expiry", "gnss-fix-expiry", "millis-wrap"):
            with self.subTest(case=case):
                subprocess.run([str(self.executable), case], check=True)

    def test_timeout_and_freshness_helpers_are_wired_into_runtime(self):
        gsm = (ROOT / "src" / "GSM.cpp").read_text(encoding="utf-8")
        main = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        timeout = function(gsm, "static uint32_t GetActiveSimCommandTimeoutMs()")
        self.assertIn("SIM7600_DATA_RESPONSE_TIMEOUT_MS", timeout)
        self.assertIn("SIM7600_MQTT_CONNECT_RESPONSE_TIMEOUT_MS", timeout)
        self.assertNotIn("SIM7600_GPS_COMMAND_MAX_BLOCK_MS", gsm)
        update = function(gsm, "void UpdateSIM7600()")
        self.assertIn("UpdateGPSFixState(false, now);", update)
        self.assertIn("ProcessSimLineChar(incoming);", update)
        self.assertIn("ServiceSimAsyncWait(now);", update)
        self.assertIn("GetActiveSimCommandPhaseTimeoutMs()", update)
        self.assertNotIn("HasCompleteGNSSInfoLine(simBuffer) || responseError", update)
        self.assertIn("RefreshGPSFixState();", function(gsm, "void GetCellularDiagnosticReport("))
        self.assertIn("RefreshGPSFixState();", function(main, "void loop()"))
        self.assertIn("last valid fix %lu s ago", function(gsm, "void GetCellularDiagnosticReport("))
        log_transfer = function(main, "if (logTransferActive)\n    {")
        self.assertIn("UpdateSIM7600(GPS);", log_transfer)
        self.assertIn("UpdateSIM7600();", log_transfer)


if __name__ == "__main__":
    unittest.main()
