/// HEADER
#include <csapex/utility/ticker.h>

/// PROJECT
#include <csapex/utility/thread.h>

using namespace csapex;

Ticker::Ticker()
    : tick_thread_running_(false),
      tick_enabled_(true),
      tick_rate_(30.0, false)
{

}

Ticker::~Ticker()
{
    stopTickThread();
}

void Ticker::tickLoop()
{
    while(!tick_thread_stop_) {
        tickEvent();
        tick_rate_.keepUp();
    }
}

void Ticker::startTickThread()
{
    if(!tick_thread_running_) {
        ticking_thread_ = std::thread([this]() {
            csapex::thread::set_name("ticker");

            tick_thread_running_ = true;

            tick_thread_stop_ = false;
            tickLoop();

            tick_thread_running_ = false;
        });
    }
}

void Ticker::stopTickThread()
{
    if(tick_thread_running_) {
        tick_thread_stop_ = true;
        if(ticking_thread_.joinable()) {
            ticking_thread_.join();
        }
    }
}


bool Ticker::isTickEnabled() const
{
    return tick_enabled_;
}

void Ticker::setTickEnabled(bool tick)
{
    tick_enabled_ = tick;
}

double Ticker::getTickFrequency() const
{
    return tick_rate_.getFrequency();
}
void Ticker::setTickFrequency(double f)
{
    tick_rate_.setFrequency(f);
}

void Ticker::setTickImmediate(bool immediate)
{
    tick_rate_.setImmediate(immediate);
}

bool Ticker::isImmediate() const
{
    return tick_rate_.isImmediate();
}
