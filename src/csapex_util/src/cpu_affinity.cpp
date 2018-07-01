/// HEADER
#include <csapex/utility/cpu_affinity.h>

/// SYSTEM
#include <thread>
#include <iostream>

using namespace csapex;

CpuAffinity::CpuAffinity(std::vector<bool> affinity) : affinity_(affinity)
{
    resize();

    update();
}

CpuAffinity::CpuAffinity()
{
    setToDefault();

    update();
}

CpuAffinity::CpuAffinity(const CpuAffinity& other) : affinity_(other.affinity_)
{
}

CpuAffinity::CpuAffinity(CpuAffinity&& other) : affinity_(other.affinity_)
{
}

CpuAffinity& CpuAffinity::operator=(const CpuAffinity& other)
{
    set(other.get());

    update();

    return *this;
}

unsigned CpuAffinity::getNumCpus() const
{
    return std::thread::hardware_concurrency();
}

std::vector<bool> CpuAffinity::get() const
{
    return affinity_;
}

void CpuAffinity::update()
{
    affinity_changed(this);
}

void CpuAffinity::set(const std::vector<bool>& affinity)
{
    if (affinity != affinity_) {
        affinity_ = affinity;
        resize();

        update();
    }
}

bool CpuAffinity::isCpuUsed(unsigned cpu) const
{
    return affinity_.at(cpu);
}

void CpuAffinity::toggleCpu(unsigned cpu)
{
    affinity_.at(cpu) = !affinity_.at(cpu);

    update();
}

void CpuAffinity::resize()
{
    if (isValid()) {
        // if the current affinity contains at least one cpu, fill with false
        affinity_.resize(getNumCpus(), false);
    } else {
        // if the current affinity contains no cpu, generate default
        setToDefault();
    }
}

void CpuAffinity::setToDefault()
{
    // TODO: make part of settings
    affinity_.clear();
    affinity_.resize(getNumCpus(), true);
}

bool CpuAffinity::isValid()
{
    std::size_t n = std::min<unsigned int>(affinity_.size(), getNumCpus());
    for (std::size_t i = 0; i < n; ++i) {
        if (affinity_[i]) {
            return true;
        }
    }

    return false;
}
