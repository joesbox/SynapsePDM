"""Compile the production RTC sync function with mocked clock/timezone/storage."""

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
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#define CHECK(c) do { if (!(c)) { std::fprintf(stderr, "%d: %s\n", __LINE__, #c); std::exit(1); } } while (0)
constexpr int RTC_MIN_FULL_YEAR = 2024;
constexpr int RTC_MAX_FULL_YEAR = 2099;
constexpr int RTC_MIN_VALID_YEAR = 24;
struct {
    time_t epoch = 0;
    uint32_t milliseconds = 0;
    unsigned writes = 0;
    unsigned snapshotReads = 0;
    bool set = true;
    bool isTimeSet() { return set; }
    uint8_t getYear() { return set ? 26 : 0; }
    time_t getEpoch() { return epoch; }
    time_t getEpoch(uint32_t *subSeconds) {
        ++snapshotReads;
        *subSeconds = milliseconds;
        time_t captured = epoch;
        ++epoch;
        milliseconds = 0;
        return captured;
    }
    void setEpoch(time_t value) { epoch = value; milliseconds = 0; set = true; ++writes; }
} rtc;
struct TimeZoneRule { int32_t StandardOffsetMinutes = 0; int32_t DSTOffsetMinutes = 60; };
struct { TimeZoneRule TimeZone; uint8_t DSTActive = 0; } SystemParams;
bool RTCSet = true;
bool SDCardOK = true;
bool dst = false;
unsigned saves = 0, sdStarts = 0;
void InitialiseSD() { ++sdStarts; }
void QueueSystemConfigSave() { ++saves; }
void SanitizeTimeZoneRule(TimeZoneRule *) {}
bool IsDSTActiveForUtcEpoch(time_t) { return dst; }
bool DetermineDstStateFromLocalEpoch(time_t) { return dst; }
char logLine[128] = {};
bool AppendFormattedText(char *buffer, size_t capacity, size_t *index, const char *format, ...) {
    va_list args;
    va_start(args, format);
    int count = vsnprintf(buffer + *index, capacity - *index, format, args);
    va_end(args);
    CHECK(count > 0 && static_cast<size_t>(count) < capacity - *index);
    *index += count;
    return true;
}
"""

CASES = r"""
static bool Sync(time_t utc) {
    const tm value = *gmtime(&utc);
    return ApplyUtcRtcDateTime(value.tm_year + 1900, value.tm_mon + 1, value.tm_mday,
                               value.tm_hour, value.tm_min, value.tm_sec);
}
int main(int argc, char **argv) {
    CHECK(argc == 2);
    const char *test = argv[1];
    time_t start = BuildEpochFromDateTime(2026, 10, 27, 7, 46, 49);
    rtc.epoch = start;
    rtc.milliseconds = 902;
    if (!std::strcmp(test, "stale-repeat")) {
        CHECK(Sync(start));
        rtc.epoch = start + 1;
        rtc.milliseconds = 101;
        for (int i = 0; i < 10; ++i) CHECK(Sync(start));
        CHECK(rtc.epoch == start + 1 && rtc.milliseconds == 101 && rtc.writes == 0);
    } else if (!std::strcmp(test, "near-behind") || !std::strcmp(test, "near-ahead")) {
        int delta = !std::strcmp(test, "near-behind") ? -1 : 1;
        CHECK(Sync(start + delta));
        CHECK(rtc.epoch == start && rtc.milliseconds == 902 && rtc.writes == 0);
    } else if (!std::strcmp(test, "large-behind") || !std::strcmp(test, "large-ahead")) {
        int delta = !std::strcmp(test, "large-behind") ? -2 : 2;
        CHECK(Sync(start + delta));
        CHECK(rtc.epoch == start + delta && rtc.writes == 1);
    } else if (!std::strcmp(test, "out-of-order")) {
        CHECK(Sync(start));
        CHECK(Sync(start - 60));
        CHECK(rtc.epoch == start && rtc.milliseconds == 902 && rtc.writes == 0);
    } else if (!std::strcmp(test, "unset") || !std::strcmp(test, "unusable")) {
        RTCSet = false;
        if (!std::strcmp(test, "unusable")) { RTCSet = true; rtc.set = false; }
        SDCardOK = false;
        CHECK(Sync(start));
        CHECK(RTCSet && rtc.set && rtc.writes == 1 && sdStarts == 1);
    } else if (!std::strcmp(test, "timezone")) {
        SystemParams.TimeZone.StandardOffsetMinutes = 330;
        CHECK(Sync(start));
        CHECK(rtc.epoch == start + 330 * 60 && rtc.writes == 1);
    } else if (!std::strcmp(test, "dst")) {
        dst = true;
        rtc.epoch = start + 3600;
        CHECK(Sync(start));
        CHECK(rtc.epoch == start + 3600 && SystemParams.DSTActive == 1 && saves == 1);
        CHECK(Sync(start + 1));
        CHECK(rtc.writes == 1 && saves == 1);
    } else if (!std::strcmp(test, "invalid")) {
        CHECK(!ApplyUtcRtcDateTime(2026, 0, 27, 7, 46, 49));
        CHECK(!ApplyUtcRtcDateTime(2026, 10, 27, 24, 46, 49));
        CHECK(Sync(start));
        CHECK(rtc.writes == 0);
    } else if (!std::strcmp(test, "midnight")) {
        start = BuildEpochFromDateTime(2026, 12, 31, 23, 59, 59);
        rtc.epoch = start;
        CHECK(Sync(start));
        rtc.epoch = start + 1;
        CHECK(Sync(start + 1));
        CHECK(rtc.writes == 0 && rtc.milliseconds == 902);
    } else if (!std::strcmp(test, "manual")) {
        CHECK(Sync(start));
        CHECK(ApplyRtcDateTime(2026, 10, 27, 7, 40, 0));
        CHECK(rtc.epoch == BuildEpochFromDateTime(2026, 10, 27, 7, 40, 0));
        CHECK(rtc.writes == 1);
    } else if (!std::strcmp(test, "ten-hz")) {
        // Fresh whole-second GPS time lags the RTC by 200 ms; never restart its phase.
        for (unsigned sample = 0; sample < 3000; ++sample) {
            unsigned elapsedMs = 900 + sample * 100;
            rtc.epoch = start + elapsedMs / 1000;
            rtc.milliseconds = elapsedMs % 1000;
            if (sample % 10 == 0) CHECK(Sync(start + (elapsedMs - 200) / 1000));
            CHECK(rtc.writes == 0);
        }
    } else if (!std::strcmp(test, "timestamp-snapshot")) {
        rtc.epoch = BuildEpochFromDateTime(2026, 12, 31, 23, 59, 59);
        rtc.milliseconds = 999;
        CaptureLogTimestamp();
        CHECK(!std::strcmp(logLine, "2026-12-31,23:59:59.999,"));
        CaptureLogTimestamp();
        CHECK(!std::strcmp(logLine, "2027-01-01,00:00:00.000,"));
        CHECK(rtc.snapshotReads == 2);
        rtc.epoch = BuildEpochFromDateTime(2028, 2, 29, 23, 59, 59);
        rtc.milliseconds = 999;
        CaptureLogTimestamp();
        CHECK(!std::strcmp(logLine, "2028-02-29,23:59:59.999,"));
        CaptureLogTimestamp();
        CHECK(!std::strcmp(logLine, "2028-03-01,00:00:00.000,"));
    } else CHECK(false);
    CHECK(saves <= 1);
}
"""


class RtcSyncTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = (ROOT / "src" / "System.cpp").read_text(encoding="utf-8")
        signatures = [
            "static int32_t DaysFromCivil(",
            "static time_t BuildEpochFromDateTime(",
            "static void UpdateRtcFromLocalEpoch(",
            "bool IsValidRtcDateTime(",
            "bool HasUsableRtcTime(",
            "bool ApplyRtcDateTime(",
            "bool ApplyUtcRtcDateTime(",
        ]
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        path = Path(cls.temp.name)
        cpp = path / "rtc_sync.cpp"
        storage = (ROOT / "src" / "Storage.cpp").read_text(encoding="utf-8")
        start = storage.index("        uint32_t logMilliseconds = 0;")
        end = storage.index("        float loggedLat", start)
        snapshot = "static void CaptureLogTimestamp() { size_t logLineIndex = 0;\n" + storage[start:end] + "}\n"
        cpp.write_text(HARNESS + "\n".join(function(source, item) for item in signatures) + snapshot + CASES, encoding="utf-8")
        cls.executable = path / "rtc_sync.exe"
        subprocess.run(
            [sys.executable, "-m", "ziglang", "c++", "-std=c++17", "-O2",
             str(cpp), "-o", str(cls.executable)],
            check=True,
        )

    def test_clock_scenarios(self):
        for case in ("stale-repeat", "near-behind", "near-ahead", "large-behind",
                     "large-ahead", "out-of-order", "unset", "unusable",
                     "timezone", "dst", "invalid", "midnight", "manual", "ten-hz",
                     "timestamp-snapshot"):
            with self.subTest(case=case):
                subprocess.run([str(self.executable), case], check=True)

    def test_only_fresh_accepted_fixes_sync_clock(self):
        main = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
        gsm = (ROOT / "src" / "GSM.cpp").read_text(encoding="utf-8")
        self.assertNotIn("ApplyUtcRtcDateTime(", main)
        parser = gsm[gsm.index("    ParsedGPSFix candidate = {};"):]
        self.assertLess(parser.index("AcceptCandidateFix(candidate, now);"),
                        parser.index("ApplyUtcRtcDateTime("))
        self.assertIn("if (hasFreshDate && hasFreshTime)", parser)
        self.assertIn("hasFreshDate = sscanf", parser)
        self.assertIn("hasFreshTime = sscanf", parser)


if __name__ == "__main__":
    unittest.main()
