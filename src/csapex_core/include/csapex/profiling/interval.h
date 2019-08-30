#ifndef INTERVAL_H
#define INTERVAL_H

/// COMPONENT
#include <csapex_core/csapex_profiling_export.h>
#include <csapex/serialization/serializable.h>

/// SYSTEM
#include <map>
#include <chrono>
#include <memory>
#include <vector>

namespace csapex
{
class Timer;

class CSAPEX_PROFILING_EXPORT Interval : public Serializable
{
    friend class Timer;

public:
    typedef std::shared_ptr<Interval> Ptr;

protected:
    CLONABLE_IMPLEMENTATION(Interval);

public:
    Interval(const std::string& name);

    long getStartMs() const;
    long getEndMs() const;
    long getStartMicro() const;
    long getEndMicro() const;

    void start();
    void stop();

    bool isStopped() const;

    std::string name() const;

    double lengthMs() const;
    double lengthSubMs() const;

    void entries(std::vector<std::pair<std::string, double> >& out) const;

    void setActive(bool active);
    bool isActive() const;

    virtual void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    virtual void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    static Ptr makeEmpty();

private:
    Interval();

public:
    std::map<std::string, Interval::Ptr> sub;

private:
    std::string name_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
    std::chrono::time_point<std::chrono::high_resolution_clock> end_;
    long length_micro_seconds_;

    bool active_;
    bool stopped_;
};

}  // namespace csapex

#endif  // INTERVAL_H
