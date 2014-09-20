/// HEADER
#include <utils_param/interval_parameter.h>

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

boost::any IntervalParameter::get_unsafe() const
{
    if(is<std::pair<int, int> >()) {
        return std::make_pair(boost::any_cast<int>(values_.first), boost::any_cast<int>(values_.second));
    } else {
        return std::make_pair(boost::any_cast<double>(values_.first), boost::any_cast<double>(values_.second));
    }
}


void IntervalParameter::set_unsafe(const boost::any &v)
{
    if(is<std::pair<int, int> >()) {
        values_ = boost::any_cast<std::pair<int, int> > (v);
    } else {
        values_ = boost::any_cast<std::pair<double, double> > (v);
    }
}


void IntervalParameter::doSetValueFrom(const Parameter &other)
{
    const IntervalParameter* interval = dynamic_cast<const IntervalParameter*>(&other);
    if(interval) {
        bool change = false;
        if(type() == typeid(std::pair<int, int>)) {
            change = boost::any_cast<int>(values_.first) != boost::any_cast<int>(interval->values_.first) &&
                    boost::any_cast<int>(values_.second) != boost::any_cast<int>(interval->values_.second);
        } else if(type() == typeid(std::pair<double, double>)) {
            change = boost::any_cast<double>(values_.first) != boost::any_cast<double>(interval->values_.first) &&
                    boost::any_cast<double>(values_.second) != boost::any_cast<double>(interval->values_.second);
        }
        if(change) {
            values_ = interval->values_;
            triggerChange();
        }
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void IntervalParameter::doClone(const Parameter &other)
{
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
    if(values_.first.type() == typeid(int)) {
        n["int"][0] = boost::any_cast<int> (values_.first);
        n["int"][1] = boost::any_cast<int> (values_.second);

    } else if(values_.first.type() == typeid(double)) {
        n["double"][0] = boost::any_cast<double> (values_.first);
        n["double"][1] = boost::any_cast<double> (values_.second);
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
    if(!n["name"].IsDefined()) {
        return;
    }

    name_ = n["name"].as<std::string>();

    if(n["int"].IsDefined()) {
        values_ = __read<int>(n["int"]);

    } else if(n["double"].IsDefined()) {
        values_ = __read<double>(n["double"]);

    } else {
        throw std::runtime_error("cannot read interval");
    }
}
