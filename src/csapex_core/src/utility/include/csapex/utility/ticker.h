#ifndef TICKER_H
#define TICKER_H

/// PROJECT
#include <csapex/utility/rate.h>

/// SYSTEM
#include <thread>
#include <atomic>
#include <mutex>

namespace csapex
{

class Ticker
{
public:
    Ticker();
    virtual ~Ticker();

protected:
    void startTickThread();
    void stopTickThread();

    bool isTickEnabled() const;
    void setTickEnabled(bool tick);

    void setTickFrequency(double f);
    double getTickFrequency() const;

    void setTickImmediate(bool immediate);
    bool isImmediate() const;


protected:
    virtual void tickEvent() = 0;

private:
    void tickLoop();

private:
    std::thread ticking_thread_;

    std::atomic<bool> tick_thread_running_;
    std::atomic<bool> tick_thread_stop_;

    bool tick_enabled_;
    Rate tick_rate_;
};

}

#endif // TICKER_H
