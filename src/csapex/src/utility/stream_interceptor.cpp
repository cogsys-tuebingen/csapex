/// HEADER
#include <csapex/utility/stream_interceptor.h>

/// PROJECT
#include <csapex/utility/thread.h>

/// SYSTEM
#include <iostream>
#include <stdio.h>
#include <thread>

#ifdef WIN32
#else
#include <unistd.h>
#endif


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
    std::unique_lock<std::mutex>(cin_mutex_);

    std::string in = cin_.str();
    cin_.str(std::string());

    return in;
}

namespace {
bool inputAvailable()
{
#ifdef WIN32
#else
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return (FD_ISSET(0, &fds));
#endif
	return false;
}
}

void StreamInterceptor::run() {
    csapex::thread::set_name("stream_interceptor");

#ifdef WIN32
	if(false) {
#else
    if (isatty(fileno(stdin))) {
#endif
        // cin is a terminal -> not polling
        return;
    }

    running_ = true;
    std::string line;
    while(!stop_) {
        if(!std::cin.good()) {
            stop_ = true;
            continue;
        }

        if(inputAvailable()) {
            in_getline_ = true;
            std::getline(std::cin,line);
            in_getline_ = false;

            if(stop_) {
                continue;
            }

            if(line[0] != '\0') {
                had_input_ = true;

                std::unique_lock<std::mutex> lock(cin_mutex_);
                cin_ << line << '\n';
                continue;
            }
        }

        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
}

StreamInterceptor::~StreamInterceptor()
{
    shutdown();

    if(thread_.joinable()) {
        thread_.join();
    }
}


StreamInterceptor::StreamInterceptor()
    : cout(std::cout.rdbuf()), cerr(std::cerr.rdbuf()), clog(std::clog.rdbuf()),
      running_(false),  in_getline_(false), had_input_(false)
{
    clog_global_ = std::clog.rdbuf();
    cout_global_ = std::cout.rdbuf();

    cout.rdbuf(cout_global_);

    fake_cout_.str(std::string());

    std::cout.rdbuf(std::clog.rdbuf());
}

bool StreamInterceptor::isRunning() const
{
    return running_;
}

void StreamInterceptor::start()
{
    if(!isRunning()) {
        thread_ = std::thread([this]() {
            run();
        });
    }
}

void StreamInterceptor::shutdown()
{
    if(running_) {
        stop_ = true;
        thread_.join();
        running_ = false;
    }
}
