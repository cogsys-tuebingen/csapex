#ifndef CPU_AFFINITY_H
#define CPU_AFFINITY_H

/// PROJECT
#include <csapex/utility/slim_signal.hpp>

/// SYSTEM
#include <vector>

namespace csapex
{
class CpuAffinity
{
public:
    CpuAffinity();
    CpuAffinity(const CpuAffinity& other);
    CpuAffinity(CpuAffinity&& other);
    CpuAffinity(std::vector<bool> affinity);

    CpuAffinity& operator=(const CpuAffinity& other);

    std::vector<bool> get() const;
    void set(const std::vector<bool>& affinity);

    bool isCpuUsed(unsigned cpu) const;

    void toggleCpu(unsigned cpu);

    unsigned getNumCpus() const;

public:
    slim_signal::Signal<void(const CpuAffinity* affinity)> affinity_changed;

private:
    void setToDefault();
    void resize();
    bool isValid();
    void update();

private:
    std::vector<bool> affinity_;
};

}  // namespace csapex

#endif  // CPU_AFFINITY_H
