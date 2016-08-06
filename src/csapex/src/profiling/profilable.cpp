/// HEADER
#include <csapex/profiling/profilable.h>

/// COMPONENT
#include <csapex/profiling/profiler.h>

using namespace csapex;

Profilable::Profilable()
    : profiler_(std::make_shared<Profiler>())
{

}

Profilable::Profilable(std::shared_ptr<Profiler> profiler)
    : profiler_(profiler)
{

}

void Profilable::useProfiler(std::shared_ptr<Profiler> profiler)
{
    apex_assert_hard(profiler);
    profiler_ = profiler;
}

std::shared_ptr<Profiler> Profilable::getProfiler()
{
    return profiler_;
}
