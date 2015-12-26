#ifndef INTERLUDE_HPP
#define INTERLUDE_HPP

/// SYSTEM
#include <sstream>

#define CONCATENATE_DETAIL(x, y) x##y
#define CONCATENATE(x, y) CONCATENATE_DETAIL(x, y)
#define MAKE_UNIQUE(x) CONCATENATE(x, __LINE__)

#define INTERLUDE(name) \
    csapex::Timer::Interlude::Ptr MAKE_UNIQUE(__interlude__) = profiling_timer_ ? \
    profiling_timer_->step(name) \
    : \
    csapex::Timer::Interlude::Ptr(nullptr)

#define INTERLUDE_STREAM(stream) \
    std::stringstream MAKE_UNIQUE(__ss__); \
    MAKE_UNIQUE(__ss__) << stream; \
    csapex::Timer::Interlude::Ptr MAKE_UNIQUE(__interlude__) = profiling_timer_ ? \
    profiling_timer_->step(MAKE_UNIQUE(__ss__).str()) \
    : \
    csapex::Timer::Interlude::Ptr(nullptr)

#define NAMED_INTERLUDE(name) \
    csapex::Timer::Interlude::Ptr interlude_##name = profiling_timer_ ? \
    profiling_timer_->step(#name) \
    : \
    csapex::Timer::Interlude::Ptr(nullptr)
#define NAMED_INTERLUDE_INSTANCE(instance,name) \
    csapex::Timer::Interlude::Ptr interlude_##name = instance->getTimer() ? \
    instance->getTimer()->step(#name) \
    : \
    csapex::Timer::Interlude::Ptr(nullptr)

#endif // INTERLUDE_HPP

