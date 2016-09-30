/// HEADER
#include <csapex/param/interval_parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#undef NDEBUG
#include <assert.h>

using namespace csapex;
using namespace param;

IntervalParameter::IntervalParameter()
    : Parameter("noname", ParameterDescription())
{
}

IntervalParameter::IntervalParameter(const std::string &name, const ParameterDescription& description)
    : Parameter(name, description)
{
}

IntervalParameter::~IntervalParameter()
{

}


const std::type_info& IntervalParameter::type() const
{
    Lock l = lock();
    if(values_.first.type() == typeid(int)) {
        return typeid(std::pair<int,int>);
    } else if(values_.first.type() == typeid(double)) {
        return typeid(std::pair<double,double>);
    } else {
        throw std::logic_error("unknown type");
    }
}

std::string IntervalParameter::toStringImpl() const
{
    Lock l = lock();
    std::stringstream v;
    v << "[interval: ";

    if(values_.first.type() == typeid(int)) {
        v << boost::any_cast<int> (values_.first) << " : " << boost::any_cast<int> (values_.second);

    } else if(values_.first.type() == typeid(double)) {
        v << boost::any_cast<double> (values_.first) << " : " << boost::any_cast<double> (values_.second);

    }

    v << "]";

    return v.str();
}

void IntervalParameter::get_unsafe(boost::any& out) const
{
    Lock l = lock();
    if(is<std::pair<int, int> >()) {
        out = std::make_pair(boost::any_cast<int>(values_.first), boost::any_cast<int>(values_.second));
    } else {
        out = std::make_pair(boost::any_cast<double>(values_.first), boost::any_cast<double>(values_.second));
    }
}


bool IntervalParameter::set_unsafe(const boost::any &v)
{
    Lock l = lock();
    if(v.type() == typeid(std::pair<int, int>)) {
        auto val = boost::any_cast<std::pair<int, int> > (v);
        if(values_.first.empty()) {
            values_ = val;
            return true;
        }
        if(boost::any_cast<int>(values_.first) != val.first || boost::any_cast<int>(values_.second) != val.second) {
            values_ = val;
            return true;
        }
    } else {
        auto val = boost::any_cast<std::pair<double, double> > (v);
        if(values_.first.empty()) {
            values_= val;
            return true;
        }
        if(boost::any_cast<double>(values_.first) != val.first || boost::any_cast<double>(values_.second) != val.second) {
            values_ = val;
            return true;
        }
    }

    return false;
}


void IntervalParameter::doSetValueFrom(const Parameter &other)
{
    Lock l = lock();
    const IntervalParameter* interval = dynamic_cast<const IntervalParameter*>(&other);
    if(interval) {
        bool change = false;
        if(type() == typeid(std::pair<int, int>)) {
            change = boost::any_cast<int>(values_.first) != boost::any_cast<int>(interval->values_.first) ||
                    boost::any_cast<int>(values_.second) != boost::any_cast<int>(interval->values_.second);
        } else if(type() == typeid(std::pair<double, double>)) {
            change = boost::any_cast<double>(values_.first) != boost::any_cast<double>(interval->values_.first) ||
                    boost::any_cast<double>(values_.second) != boost::any_cast<double>(interval->values_.second);
        }
        values_ = interval->values_;
        if(!interval->max_.empty()) {
            max_ = interval->max_;
        }
        if(!interval->min_.empty()) {
            min_ = interval->min_;
        }
        if(!interval->step_.empty()) {
            step_ = interval->step_;
        }

        if(change) {
            triggerChange();
        }
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void IntervalParameter::doClone(const Parameter &other)
{
    Lock l = lock();
    const IntervalParameter* interval = dynamic_cast<const IntervalParameter*>(&other);
    if(interval) {
        values_ = interval->values_;
        min_ = interval->min_;
        max_ = interval->max_;
        def_ = interval->def_;
        step_ = interval->step_;
    } else {
        throw std::runtime_error("bad clone, invalid types");
    }
}

void IntervalParameter::doSerialize(YAML::Node& n) const
{
    Lock l = lock();
    if(values_.first.type() == typeid(int)) {
        n["int"][0] = boost::any_cast<int> (values_.first);
        n["int"][1] = boost::any_cast<int> (values_.second);
        if(!min_.empty())
            n["min"] = boost::any_cast<int> (min_);
        if(!max_.empty())
            n["max"] = boost::any_cast<int> (max_);
        if(!step_.empty())
            n["step"] = boost::any_cast<int> (step_);

    } else if(values_.first.type() == typeid(double)) {
        n["double"][0] = boost::any_cast<double> (values_.first);
        n["double"][1] = boost::any_cast<double> (values_.second);
        if(!min_.empty())
            n["min"] = boost::any_cast<double> (min_);
        if(!max_.empty())
            n["max"] = boost::any_cast<double> (max_);
        if(!step_.empty())
            n["step"] = boost::any_cast<double> (step_);
    }
}

namespace {
template <typename T>
std::pair<T,T> __read(const YAML::Node& n) {
    std::pair<T,T> v;
    assert(n.Type() == YAML::NodeType::Sequence);
    v.first = n[0].as<T>();
    v.second = n[1].as<T>();
    return v;
}
}

void IntervalParameter::doDeserialize(const YAML::Node& n)
{
    if(n["int"].IsDefined()) {
        values_ = __read<int>(n["int"]);
        if(n["min"].IsDefined())
            min_ = n["min"].as<int>();
        if(n["max"].IsDefined())
            max_ = n["max"].as<int>();
        if(n["step"].IsDefined())
            step_ = n["step"].as<int>();

    } else if(n["double"].IsDefined()) {
        values_ = __read<double>(n["double"]);
        if(n["min"].IsDefined())
            min_ = n["min"].as<double>();
        if(n["max"].IsDefined())
            max_ = n["max"].as<double>();
        if(n["step"].IsDefined())
            step_ = n["step"].as<double>();

    } else {
        throw std::runtime_error("cannot read interval");
    }
}
