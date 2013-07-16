#ifndef STREAM_INTERCEPTOR_H
#define STREAM_INTERCEPTOR_H

/// SYSTEM
#include <QWidget>
#include <sstream>

class StreamInterceptor
{
public:
    static StreamInterceptor& instance();

    std::string getLatest();

private:
    StreamInterceptor();

    std::streambuf *clog_global_;
    std::streambuf *cout_global_;

    std::stringstream fake_cout_;
};

#endif // STREAM_INTERCEPTOR_H
