#ifndef INTERLUDE_HPP
#define INTERLUDE_HPP

/// SYSTEM
#include <sstream>

#define CONCATENATE_DETAIL(x, y) x##y
#define CONCATENATE(x, y) CONCATENATE_DETAIL(x, y)
#define MAKE_UNIQUE(x) CONCATENATE(x, __LINE__)

#define INTERLUDE(name) \
    csapex::Interlude::Ptr MAKE_UNIQUE(__interlude__) = profiling_timer_->isEnabled() ? \
    profiling_timer_->step(name) \
    : \
    csapex::Interlude::Ptr(nullptr)

#define INTERLUDE_STREAM(stream) \
    std::stringstream MAKE_UNIQUE(__ss__); \
    MAKE_UNIQUE(__ss__) << stream; \
    csapex::Interlude::Ptr MAKE_UNIQUE(__interlude__) = profiling_timer_->isEnabled() ? \
    profiling_timer_->step(MAKE_UNIQUE(__ss__).str()) \
    : \
    csapex::Interlude::Ptr(nullptr)

#define NAMED_INTERLUDE(name) \
    csapex::Interlude::Ptr interlude_##name = profiling_timer_->isEnabled() ? \
    profiling_timer_->step(#name) \
    : \
    csapex::Interlude::Ptr(nullptr)
#define NAMED_INTERLUDE_INSTANCE(instance,name) \
    csapex::Interlude::Ptr interlude_##name = instance->getTimer()->isEnabled() ? \
    instance->getTimer()->step(#name) \
    : \
    csapex::Interlude::Ptr(nullptr)

#endif // INTERLUDE_HPP

