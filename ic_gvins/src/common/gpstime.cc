/* IC-GVINS navigation time conversion. */

#include "ic_gvins/common/gpstime.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace {

constexpr int64_t NS_PER_SEC     = 1'000'000'000LL;
constexpr int64_t SEC_PER_WEEK   = 604'800LL;
constexpr int64_t UNIX_GPS_EPOCH = 315'964'800LL;

// Leap-second table adapted from open_nav/nav_core revision
// 882e8ba12889d04cbee0cf4a267518360d9df28d.
struct LeapSecond {
    int64_t unix_sec;
    int64_t gps_sec;
    int offset;
};

constexpr std::array<LeapSecond, 19> LEAP_SECONDS = {{{0LL, 0LL, 0},
                                                      {362793600LL, 46828801LL, 1},
                                                      {394329600LL, 78364802LL, 2},
                                                      {425865600LL, 109900803LL, 3},
                                                      {489024000LL, 173059204LL, 4},
                                                      {567993600LL, 252028805LL, 5},
                                                      {631152000LL, 315187206LL, 6},
                                                      {662688000LL, 346723207LL, 7},
                                                      {709948800LL, 393984008LL, 8},
                                                      {741484800LL, 425520009LL, 9},
                                                      {773020800LL, 457056010LL, 10},
                                                      {820454400LL, 504489611LL, 11},
                                                      {867715200LL, 551750412LL, 12},
                                                      {915148800LL, 599184013LL, 13},
                                                      {1136073600LL, 820108814LL, 14},
                                                      {1230768000LL, 914803215LL, 15},
                                                      {1341100800LL, 1025136016LL, 16},
                                                      {1435708800LL, 1119744017LL, 17},
                                                      {1483228800LL, 1167264018LL, 18}}};

int64_t secondsToNanoseconds(double seconds) {
    return static_cast<int64_t>(std::llround(static_cast<long double>(seconds) * NS_PER_SEC));
}

void splitNanoseconds(int64_t nanoseconds, int64_t &seconds, int64_t &remainder) {
    seconds   = nanoseconds / NS_PER_SEC;
    remainder = nanoseconds % NS_PER_SEC;
    if (remainder < 0) {
        --seconds;
        remainder += NS_PER_SEC;
    }
}

int64_t floorDivide(int64_t dividend, int64_t divisor) {
    int64_t quotient        = dividend / divisor;
    const int64_t remainder = dividend % divisor;
    if (remainder != 0 && ((remainder < 0) != (divisor < 0))) {
        --quotient;
    }
    return quotient;
}

int leapSecondsFromUnix(int64_t seconds) {
    for (auto it = LEAP_SECONDS.rbegin(); it != LEAP_SECONDS.rend(); ++it) {
        if (seconds >= it->unix_sec) {
            return it->offset;
        }
    }
    return 0;
}

int leapSecondsFromGps(int64_t seconds) {
    for (auto it = LEAP_SECONDS.rbegin(); it != LEAP_SECONDS.rend(); ++it) {
        if (seconds >= it->gps_sec) {
            return it->offset;
        }
    }
    return 0;
}

bool isLeapSecond(int64_t gps_seconds) {
    for (std::size_t i = 1; i < LEAP_SECONDS.size(); ++i) {
        if (gps_seconds == LEAP_SECONDS[i].gps_sec - 1) {
            return true;
        }
    }
    return false;
}

void gpsNanosecondsToWeekTime(int64_t gps_nanoseconds, int &week, double &sow) {
    int64_t gps_seconds = 0;
    int64_t nanoseconds = 0;
    splitNanoseconds(gps_nanoseconds, gps_seconds, nanoseconds);
    const int64_t gps_week = floorDivide(gps_seconds, SEC_PER_WEEK);
    week                   = static_cast<int>(gps_week);
    sow = static_cast<double>(gps_seconds - gps_week * SEC_PER_WEEK) + static_cast<double>(nanoseconds) * 1e-9;
}

} // namespace

void GpsTime::gps2unix(int week, double sow, double &unixs) {
    const int64_t gps_nanoseconds = static_cast<int64_t>(week) * SEC_PER_WEEK * NS_PER_SEC + secondsToNanoseconds(sow);
    int64_t gps_seconds           = 0;
    int64_t nanoseconds           = 0;
    splitNanoseconds(gps_nanoseconds, gps_seconds, nanoseconds);

    int offset = leapSecondsFromGps(gps_seconds);
    if (isLeapSecond(gps_seconds)) {
        ++offset;
    }
    unixs = static_cast<double>(UNIX_GPS_EPOCH + gps_seconds - offset) + static_cast<double>(nanoseconds) * 1e-9;
}

void GpsTime::unix2gps(double unixs, int &week, double &sow) {
    const int64_t unix_nanoseconds = secondsToNanoseconds(unixs);
    int64_t unix_seconds           = 0;
    int64_t nanoseconds            = 0;
    splitNanoseconds(unix_nanoseconds, unix_seconds, nanoseconds);
    const int64_t gps_nanoseconds =
        (unix_seconds - UNIX_GPS_EPOCH + leapSecondsFromUnix(unix_seconds)) * NS_PER_SEC + nanoseconds;
    gpsNanosecondsToWeekTime(gps_nanoseconds, week, sow);
}

void GpsTime::unix2gps(int64_t unix_sec, uint32_t unix_nsec, int &week, double &sow) {
    if (unix_nsec >= static_cast<uint32_t>(NS_PER_SEC)) {
        throw std::invalid_argument("unix_nsec must be < 1e9");
    }
    const int64_t gps_nanoseconds =
        (unix_sec - UNIX_GPS_EPOCH + leapSecondsFromUnix(unix_sec)) * NS_PER_SEC + unix_nsec;
    gpsNanosecondsToWeekTime(gps_nanoseconds, week, sow);
}
