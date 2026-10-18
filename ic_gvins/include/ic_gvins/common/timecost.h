/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TIMECOST_H
#define TIMECOST_H

#include <chrono>
#include <iomanip>
#include <sstream>

class TimeCost {

public:
    TimeCost() {
        restart();
    }

    void restart() {
        start_     = Clock::now();
        is_finish_ = false;
    }

    void finish() {
        end_       = Clock::now();
        duration_  = end_ - start_;
        is_finish_ = true;
    }

    double costInSecond() {
        if (!is_finish_) {
            finish();
        }

        return duration_.count();
    }

    std::string costInSecond(const std::string &header) {
        auto cost = costInSecond();
        std::ostringstream stream;
        stream << header << " " << std::fixed << std::setprecision(6) << cost << " seconds";
        return stream.str();
    }

    double costInMillisecond() {
        if (!is_finish_) {
            finish();
        }
        return std::chrono::duration<double, std::milli>(duration_).count();
    }

    std::string costInMillisecond(const std::string &header) {
        auto cost = costInMillisecond();
        std::ostringstream stream;
        stream << header << " " << std::fixed << std::setprecision(3) << cost << " milliseconds";
        return stream.str();
    }

private:
    using Clock = std::chrono::steady_clock;

    Clock::time_point                     start_, end_;
    std::chrono::duration<double> duration_;

    bool is_finish_{false};
};

#endif // TIMECOST_H
