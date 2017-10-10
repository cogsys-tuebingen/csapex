/// HEADER
#include <csapex/profiling/interval.h>

/// PROJECT
#include <csapex/serialization/serialization_buffer.h>

using namespace csapex;

Interval::Interval(const std::string &name)
    : name_(name), length_micro_seconds_(0), active_(false)
{
    start();
}

Interval::Interval()
    : Interval("empty")
{

}

bool Interval::isActive() const
{
    return active_;
}

void Interval::setActive(bool active)
{
    active_ = active;
}

void Interval::entries(std::vector<std::pair<std::string, double> > &out) const
{
    out.push_back(std::make_pair(name_, lengthMs()));
    for(auto it = sub.begin(); it != sub.end(); ++it) {
        it->second->entries(out);
    }
}

double Interval::lengthMs() const
{
    return length_micro_seconds_ * 1e-3;
}

double Interval::lengthSubMs() const
{
    int sum = 0;
    for(std::map<std::string, Interval::Ptr>::const_iterator it = sub.begin(); it != sub.end(); ++it) {
        const Interval& i = *it->second;
        sum += i.lengthMs();
    }
    return sum * 1e-3;
}

std::string Interval::name() const
{
    return name_;
}

long Interval::getStartMs() const
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(start_.time_since_epoch()).count();
}

long Interval::getEndMs() const
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(end_.time_since_epoch()).count();
}

long Interval::getStartMicro() const
{
    return std::chrono::duration_cast<std::chrono::microseconds>(start_.time_since_epoch()).count();
}

long Interval::getEndMicro() const
{
    return std::chrono::duration_cast<std::chrono::microseconds>(end_.time_since_epoch()).count();
}

void Interval::start()
{
    start_ = std::chrono::high_resolution_clock::now();
}

void Interval::stop()
{
    end_ = std::chrono::high_resolution_clock::now();
    length_micro_seconds_ += std::chrono::duration_cast<std::chrono::microseconds>(end_ - start_).count();
}


std::shared_ptr<Clonable> Interval::makeEmptyClone() const
{
    return std::shared_ptr<Interval>(new Interval);
}

void Interval::serialize(SerializationBuffer &data) const
{
    data << name_;

    uint64_t start = std::chrono::nanoseconds(start_.time_since_epoch()).count();
    uint64_t end = std::chrono::nanoseconds(end_.time_since_epoch()).count();
    data << start;
    data << end;

    data << length_micro_seconds_;
    data << active_;

    data << sub;
}
void Interval::deserialize(SerializationBuffer& data)
{
    data >> name_;

    uint64_t start;
    uint64_t end;
    data >> start;
    data >> end;

    using tp = std::chrono::time_point<std::chrono::high_resolution_clock>;

    start_ = tp(std::chrono::nanoseconds(start));
    end_ = tp(std::chrono::nanoseconds(end));

    data >> length_micro_seconds_;
    data >> active_;

    data >> sub;
}
