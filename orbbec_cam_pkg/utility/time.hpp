#pragma once

#include <iostream>
#include <thread>
#include <chrono>
#include <boost/thread.hpp>
#include <unistd.h>
#include <boost/chrono.hpp>
#include <sys/time.h>

namespace CGU
{
    #ifdef USE_BOOST
        inline void SLEEP(uint64_t ms){boost::this_thread::sleep(boost::posix_time::milliseconds(ms));}
        inline uint64_t getNs(){return boost::chrono::high_resolution_clock::now().time_since_epoch().count();}
        inline uint64_t getMs(){return boost::chrono::duration_cast<boost::chrono::milliseconds>(boost::chrono::high_resolution_clock::now().time_since_epoch()).count();}
        inline uint64_t getUs(){return boost::chrono::duration_cast<boost::chrono::microseconds>(boost::chrono::high_resolution_clock::now().time_since_epoch()).count();}
    #elif defined(USE_SYSTEM)
        inline void SLEEP(uint64_t ms){usleep(ms * 1000);}
        inline uint64_t getNs(){
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            return (static_cast<long long>(ts.tv_sec) * 1000000000LL + static_cast<long long>(ts.tv_nsec));
        }
        inline uint64_t getMs(){
            struct timeval tv;
            gettimeofday(&tv, nullptr);
            return (static_cast<long long>(tv.tv_sec) * 1000LL + static_cast<long long>(tv.tv_usec) / 1000LL);
        }
        inline uint64_t getUs(){
            struct timeval tv;
            gettimeofday(&tv, nullptr);
            return (static_cast<long long>(tv.tv_sec) * 1000000LL + static_cast<long long>(tv.tv_usec));
        }
    #else
        inline void SLEEP(uint64_t ms){std::this_thread::sleep_for(std::chrono::milliseconds(ms));}
        inline uint64_t getNs(){return std::chrono::high_resolution_clock::now().time_since_epoch().count();}
        inline uint64_t getMs(){return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();}
        inline uint64_t getUs(){return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();}
    #endif
}