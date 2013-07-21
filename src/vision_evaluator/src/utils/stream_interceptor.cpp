/// HEADER
#include "stream_interceptor.h"

/// SYSTEM
#include <iostream>
#include <QtConcurrentRun>

bool StreamInterceptor::running = true;

namespace {
struct QSleepThread : public QThread{
    static void sleep(unsigned long t) {
        QThread::sleep(t);
    }
    static void msleep(unsigned long t) {
        QThread::msleep(t);
    }
    static void usleep(unsigned long t) {
        QThread::usleep(t);
    }
};
}

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
std::string StreamInterceptor::cin()
{
    cin_mutex.lock();
    std::string in = cin_.str();
    cin_.str(std::string());
    cin_mutex.unlock();

    return in;
}

StreamInterceptor::StreamInterceptor()
    : cout(std::cout.rdbuf()), cerr(std::cerr.rdbuf()), clog(std::clog.rdbuf())
{
    clog_global_ = std::clog.rdbuf();
    cout_global_ = std::cout.rdbuf();

    cout.rdbuf(cout_global_);

    fake_cout_.str(std::string());

    std::cout.rdbuf(fake_cout_.rdbuf());

    QtConcurrent::run(this, &StreamInterceptor::pollCin);
    running = true;
}

void StreamInterceptor::pollCin() {
    while(running) {
        char line[256];
        std::cin.getline(line,256);

        if(line[0] != '\0') {
            cin_mutex.lock();
            cin_ << line;
            cin_mutex.unlock();
        } else {
            QSleepThread::msleep(10);
        }
    }
}

StreamInterceptor::~StreamInterceptor()
{
    running = false;
}

