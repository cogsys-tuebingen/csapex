/// HEADER
#include "stream_interceptor.h"

/// PROJECT
#include <qt_helper.hpp>

/// SYSTEM
#include <iostream>
#include <QtConcurrentRun>
#include <unistd.h>
#include <stdio.h>

StreamInterceptor& StreamInterceptor::instance()
{
    static StreamInterceptor instance;
    return instance;
}



std::string StreamInterceptor::getCout()
{
    std::string res(fake_cout_.str());

    fake_cout_.str(std::string());
    fake_cout_.clear();

    std::clog << res.c_str() << std::flush;
    return res;
}

std::string StreamInterceptor::getCerr()
{
    std::string res(fake_cerr_.str());

    fake_cerr_.str(std::string());
    fake_cerr_.clear();

    cerr << res.c_str() << std::flush;
    return res;
}

std::string StreamInterceptor::getCin()
{
    if(!worker) {
        return "";
    }

    worker->cin_mutex.lock();
    std::string in = worker->cin_.str();
    worker->cin_.str(std::string());
    worker->cin_mutex.unlock();

    return in;
}

StreamInterceptorWorker::StreamInterceptorWorker()
    : running(true),  in_getline(false), had_input(false)
{

}
StreamInterceptorWorker::~StreamInterceptorWorker()
{
}

void StreamInterceptorWorker::run() {
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
}


StreamInterceptor::StreamInterceptor()
    : cout(std::cout.rdbuf()), cerr(std::cerr.rdbuf()), clog(std::clog.rdbuf()), thread(NULL), worker(NULL)
{
    clog_global_ = std::clog.rdbuf();
    cout_global_ = std::cout.rdbuf();

    cout.rdbuf(cout_global_);

    fake_cout_.str(std::string());

    std::cout.rdbuf(fake_cout_.rdbuf());
    std::cerr.rdbuf(fake_cerr_.rdbuf());
}

void StreamInterceptor::start()
{
    thread = new QThread;
    worker = new StreamInterceptorWorker;
    worker->moveToThread(thread);
    QObject::connect(thread, SIGNAL(started()), worker, SLOT(run()));
    QObject::connect(worker, SIGNAL(finished()), thread, SLOT(quit()));
    QObject::connect(worker, SIGNAL(finished()), worker, SLOT(deleteLater()));
    QObject::connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    thread->start();
}

void StreamInterceptor::stop()
{
    if(!worker) {
        return;
    }

    worker->running = false;
    thread->quit();
}


