#ifndef TRACE_HPP
#define TRACE_HPP

/// PROJECT
#include <csapex/profiling/timer.h>
#include <csapex/profiling/trace.h>

/// SYSTEM
#include <sstream>

#define CONCATENATE_DETAIL(x, y) x##y
#define CONCATENATE(x, y) CONCATENATE_DETAIL(x, y)
#define MAKE_UNIQUE(x) CONCATENATE(x, __LINE__)

#define TRACE(name) csapex::Trace::Ptr MAKE_UNIQUE(__TRACE__) = profiling_timer_->isEnabled() ? profiling_timer_->step(name) : csapex::Trace::Ptr(nullptr)

#define TRACE_STREAM(stream)                                                                                                                                                                       \
    std::stringstream MAKE_UNIQUE(__ss__);                                                                                                                                                             \
    MAKE_UNIQUE(__ss__) << stream;                                                                                                                                                                     \
    csapex::Trace::Ptr MAKE_UNIQUE(__TRACE__) = profiling_timer_->isEnabled() ? profiling_timer_->step(MAKE_UNIQUE(__ss__).str()) : csapex::Trace::Ptr(nullptr)

#define NAMED_TRACE(name) csapex::Trace::Ptr trace_##name = profiling_timer_->isEnabled() ? profiling_timer_->step(#name) : csapex::Trace::Ptr(nullptr)
#define NAMED_TRACE_INSTANCE(instance, name) csapex::Trace::Ptr trace_##name = instance->getTimer()->isEnabled() ? instance->getTimer()->step(#name) : csapex::Trace::Ptr(nullptr)

#endif  // TRACE_HPP
