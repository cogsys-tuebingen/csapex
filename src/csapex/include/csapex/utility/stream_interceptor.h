#ifndef STREAM_INTERCEPTOR_H
#define STREAM_INTERCEPTOR_H

/// PROJECT
#include <csapex/utility/singleton.hpp>
#include <csapex/csapex_util_export.h>

/// SYSTEM
#include <mutex>
#include <atomic>
#include <sstream>
#include <thread>

namespace csapex
{

class CSAPEX_UTILS_EXPORT StreamInterceptor : public Singleton<StreamInterceptor>
{
    friend class Singleton<StreamInterceptor>;

public:
    std::string getCout();
    std::string getCerr();
    std::string getCin();

    bool isRunning() const;
    void start();
    void shutdown() override;

public:
    std::ostream cout;
    std::ostream cerr;
    std::ostream clog;


private:
    StreamInterceptor();
    ~StreamInterceptor();

    void run();

private:
    std::streambuf *clog_global_;
    std::streambuf *cout_global_;

    std::stringstream fake_cout_;
    std::stringstream fake_cerr_;

    std::thread thread_;

    std::atomic<bool> stop_;
    bool running_;

    volatile bool in_getline_;
    volatile bool had_input_;

    std::mutex cin_mutex_;
    std::stringstream cin_;
};

}

#endif // STREAM_INTERCEPTOR_H
