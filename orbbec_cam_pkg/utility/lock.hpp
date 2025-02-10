#pragma once

#include <mutex>
#include <shared_mutex>

#ifdef USE_BOOST
#include <boost/thread.hpp>
#endif 

namespace CGU
{
    #ifdef USE_BOOST
        using SCOPE_LOCK = boost::lock_guard<boost::mutex>;
        using WRITE_LOCK = boost::unique_lock<boost::shared_mutex>;
        using READ_LOCK = boost::shared_lock<boost::shared_mutex>;
    #else
        using SCOPE_LOCK = std::lock_guard<::std::mutex>;
        using WRITE_LOCK = std::unique_lock<::std::shared_mutex>;
        using READ_LOCK = std::shared_lock<::std::shared_mutex>;         
    #endif
}


