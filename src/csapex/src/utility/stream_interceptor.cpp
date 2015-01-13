/// HEADER
#include <csapex/utility/stream_interceptor.h>

/// PROJECT
#include <csapex/utility/thread.h>

/// SYSTEM
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <thread>

using namespace csapex;


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

    std::lock_guard<std::mutex>(worker->cin_mutex);

    std::string in = worker->cin_.str();
    worker->cin_.str(std::string());

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
    csapex::thread::set_name("stream_interceptor");

//    if (isatty(fileno(stdin))) {
//        std::cout << "<b>std::cin is a terminal -> not polling</b>" << std::endl;
//        return;

//    } else {
//        std::cout << "<b>std::cin is a file or pipe -> polling</b>" << std::endl;
//    }

    while(running) {
        std::string line;

        in_getline = true;
        std::getline(std::cin,line);
        in_getline = false;

        if(line[0] != '\0') {
            had_input = true;

            std::lock_guard<std::mutex> lock(cin_mutex);
            cin_ << line;
            continue;
        }

        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
}

StreamInterceptor::~StreamInterceptor()
{
}


StreamInterceptor::StreamInterceptor()
    : cout(std::cout.rdbuf()), cerr(std::cerr.rdbuf()), clog(std::clog.rdbuf()), thread(nullptr), worker(nullptr)
{
    clog_global_ = std::clog.rdbuf();
    cout_global_ = std::cout.rdbuf();

    cout.rdbuf(cout_global_);

    fake_cout_.str(std::string());

 //   std::cout.rdbuf(fake_cout_.rdbuf());
//    std::cerr.rdbuf(fake_cerr_.rdbuf());
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


