#ifndef STREAM_INTERCEPTOR_H
#define STREAM_INTERCEPTOR_H

/// PROJECT
#include <utils_plugin/singleton.hpp>

/// SYSTEM
#include <QWidget>
#include <QMutex>
#include <QThread>
#include <sstream>

namespace csapex
{

class StreamInterceptorWorker : public QObject {
    Q_OBJECT

public:
    StreamInterceptorWorker();
    ~StreamInterceptorWorker();

public Q_SLOTS:
    void run();

Q_SIGNALS:
    void finished();
    void error(QString err);

public:
    bool running;

    volatile bool in_getline;
    volatile bool had_input;

    QMutex cin_mutex;
    std::stringstream cin_;
};

class StreamInterceptor : public QObject, public Singleton<StreamInterceptor>
{
    Q_OBJECT

    friend class Singleton<StreamInterceptor>;

public:
    std::string getCout();
    std::string getCerr();
    std::string getCin();

    void start();
    void stop();

public:
    std::ostream cout;
    std::ostream cerr;
    std::ostream clog;


private:
    StreamInterceptor();
    ~StreamInterceptor();

    std::streambuf *clog_global_;
    std::streambuf *cout_global_;

    std::stringstream fake_cout_;
    std::stringstream fake_cerr_;

    QThread* thread;
    StreamInterceptorWorker* worker;
};

}

#endif // STREAM_INTERCEPTOR_H
