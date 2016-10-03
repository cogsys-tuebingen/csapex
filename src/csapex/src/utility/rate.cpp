/// HEADER
#include <csapex/utility/rate.h>

/// SYSTEM
#include <thread>

using namespace csapex;

Rate::Rate(double frequency, bool immediate)
    : frequency_(frequency),
      immediate_(immediate)
{
    last_scheduled_tick_ = std::chrono::system_clock::now();
}

Rate::Rate()
    : Rate(-1, 0)
{
}


double Rate::getEffectiveFrequency() const
{
    if(real_ticks_.empty()) {
        return 0.0;
    }

    auto duration = real_ticks_.back() - real_ticks_.front();
    double sec = std::chrono::duration_cast<std::chrono::microseconds>(duration).count() * 1e-6;

    return real_ticks_.size() / sec;
}

double Rate::getFrequency() const
{
    return frequency_;
}
void Rate::setFrequency(double f)
{
    if(frequency_ < 0.0) {
        setImmediate(true);
    } else {
        immediate_ = false;
        frequency_ = f;
    }
}

bool Rate::isImmediate() const
{
    return immediate_;
}

void Rate::setImmediate(bool immediate)
{
    if(immediate) {
        frequency_ = 10.0;
    }
    immediate_ = immediate;
}

void Rate::keepUp()
{
    std::chrono::milliseconds interval(int(1000.0 / frequency_));

    auto now = std::chrono::system_clock::now();

    auto end_of_cycle = last_scheduled_tick_ + interval;
    last_scheduled_tick_ = end_of_cycle;

    if(end_of_cycle > now) {
        std::this_thread::sleep_until(end_of_cycle);
    }
}

void Rate::tick()
{
    auto now = std::chrono::system_clock::now();
    real_ticks_.emplace_back(now);

    std::size_t N = 16;
    while(real_ticks_.size() > N) {
        auto duration = now - real_ticks_.front();
        double sec = std::chrono::duration_cast<std::chrono::microseconds>(duration).count() * 1e-6;
        if(sec < 2.0) {
            // a least amount of history should be kept for smoothing
            return;
        } else {
            // don't keep too much history, at most N
            real_ticks_.pop_front();
        }
    }
}
