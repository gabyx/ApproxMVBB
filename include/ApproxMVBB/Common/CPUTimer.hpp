// ========================================================================================
//  ApproxMVBB
//  Copyright (C) 2014 by Gabriel Nützi <nuetzig (at) imes (d0t) mavt (d0t) ethz
//  (døt) ch>
//
//  This Source Code Form is subject to the terms of the Mozilla Public
//  License, v. 2.0. If a copy of the MPL was not distributed with this
//  file, You can obtain one at http://mozilla.org/MPL/2.0/.
// ========================================================================================

#ifndef ApproxMVBB_Common_CPUTimer_hpp
#define ApproxMVBB_Common_CPUTimer_hpp

#include <chrono>

#define START_TIMER(start) auto start = std::chrono::high_resolution_clock::now();

#define STOP_TIMER_MILLI(_count_, start) \
    auto _count_ = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count();

#define STOP_TIMER_NANOSEC(_count_, start) \
    auto _count_ = std::chrono::duration<double, std::nano>(std::chrono::high_resolution_clock::now() - start).count();

#define STOP_TIMER_SEC(_count_, start) \
    auto _count_ = std::chrono::duration<double, std::ratio<1>>(std::chrono::high_resolution_clock::now() - start).count();

class CPUTimer
{
public:
    using ClockType = std::chrono::high_resolution_clock;

    inline void start()
    {
        m_start = ClockType::now();
    }

    /** returns elapsed time in nanosecond*/
    inline double elapsed()
    {
        return std::chrono::duration<double, std::nano>(ClockType::now() - m_start).count();
    }
    /** returns elapsed time in second*/
    inline double elapsedSec()
    {
        return std::chrono::duration<double, std::ratio<1>>(ClockType::now() - m_start).count();
    }
    /** returns elapsed time in minutes*/
    inline double elapsedMin()
    {
        return std::chrono::duration<double, std::ratio<60>>(ClockType::now() - m_start).count();
    }

    /** returns elapsed time in millisecond*/
    inline double elapsedMilliSec()
    {
        return std::chrono::duration<double, std::milli>(ClockType::now() - m_start).count();
    }

private:
    std::chrono::time_point<ClockType> m_start;
};

#endif  // CPUTimer_hpp
