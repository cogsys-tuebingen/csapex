/// HEADER
#include "stream_interceptor.h"

/// PROJECT
#include <qt_helper.hpp>

/// SYSTEM
#include <iostream>
#include <QtConcurrentRun>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>

bool StreamInterceptor::running = true;

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
    : cout(std::cout.rdbuf()), cerr(std::cerr.rdbuf()), clog(std::clog.rdbuf()), in_getline(false), had_input(false)
{
    clog_global_ = std::clog.rdbuf();
    cout_global_ = std::cout.rdbuf();

    cout.rdbuf(cout_global_);

    fake_cout_.str(std::string());

    std::cout.rdbuf(fake_cout_.rdbuf());

    QtConcurrent::run(this, &StreamInterceptor::pollCin);
    running = true;
}

bool StreamInterceptor::close()
{
    running = false;

    if(!in_getline) {
        return true;
    }

    if(!had_input && in_getline) {
        // if there never was a message -> kill immediately!
        kill();
    }

    return false;
}

void StreamInterceptor::kill()
{
    if(in_getline) {
        std::cerr << "FATAL: io is blocking shutdown. KILL!" << std::endl;
        ::kill(getpid(),SIGINT);
    }
}

void StreamInterceptor::pollCin() {
    if (isatty(fileno(stdin))) {
        std::cout << "<b>std::cin is a terminal -> not polling</b>" << std::endl;
        return;

    } else {
        std::cout << "<b>std::cin is a file or pipe -> polling</b>" << std::endl;
    }

    while(running) {
        std::string line;

        in_getline = true;
        std::getline(std::cin,line);
        in_getline = false;

        if(line[0] != '\0') {
            had_input = true;

            cin_mutex.lock();
            cin_ << line;
            cin_mutex.unlock();
            continue;
        }

        QtHelper::QSleepThread::msleep(10);
    }
}

StreamInterceptor::~StreamInterceptor()
{
    running = false;
}

