#ifndef STREAM_INTERCEPTOR_H
#define STREAM_INTERCEPTOR_H

/// SYSTEM
#include <QWidget>
#include <QMutex>
#include <sstream>

class StreamInterceptor
{
public:
    static StreamInterceptor& instance();

    std::string getLatest();
    std::string cin();

    bool close();
    void kill();

public:
    std::ostream cout;
    std::ostream cerr;
    std::ostream clog;


private:
    StreamInterceptor();
    ~StreamInterceptor();

    volatile bool in_getline;
    volatile bool had_input;
    static bool running;

    void pollCin();

    QMutex cin_mutex;
    std::stringstream cin_;

    std::streambuf *clog_global_;
    std::streambuf *cout_global_;

    std::stringstream fake_cout_;
};

#endif // STREAM_INTERCEPTOR_H
