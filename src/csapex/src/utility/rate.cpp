/// HEADER
#include <csapex/utility/rate.h>

/// SYSTEM
#include <thread>

using namespace csapex;

Rate::Rate(double frequency, bool immediate)
    : frequency_(frequency),
      immediate_(immediate)
{
    last_tick_ = std::chrono::system_clock::now();
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

void Rate::tick()
{
    std::chrono::milliseconds interval(int(1000.0 / frequency_));

    auto now = std::chrono::system_clock::now();

    auto end_of_cycle = last_tick_ + interval;
    last_tick_ = end_of_cycle;

    if(end_of_cycle > now) {
        std::this_thread::sleep_until(end_of_cycle);
    }
}
