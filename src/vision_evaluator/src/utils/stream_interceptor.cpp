/// HEADER
#include "stream_interceptor.h"

/// SYSTEM
#include <iostream>

StreamInterceptor& StreamInterceptor::instance()
{
    static StreamInterceptor instance;
    return instance;
}

std::string StreamInterceptor::getLatest()
{
    std::string res(fake_cout_.str());

    fake_cout_.str(std::string());
    fake_cout_.clear();

    std::clog << res.c_str() << std::flush;
    return res;
}

StreamInterceptor::StreamInterceptor()
{    
    clog_global_ = std::clog.rdbuf();
    cout_global_ = std::cout.rdbuf();

    fake_cout_.str(std::string());

    std::cout.rdbuf(fake_cout_.rdbuf());

    std::cout << "init" << std::endl;
}
