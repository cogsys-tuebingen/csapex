/// HEADER
#include <utils_param/interval_parameter.h>

using namespace param;

IntervalParameter::IntervalParameter()
    : Parameter("noname")
{
}

IntervalParameter::IntervalParameter(const std::string &name)
    : Parameter(name)
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


void IntervalParameter::doSetFrom(const Parameter &other)
{
    const IntervalParameter* interval = dynamic_cast<const IntervalParameter*>(&other);
    if(interval) {
        values_ = interval->values_;
        triggerChange();
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void IntervalParameter::doWrite(YAML::Emitter& e) const
{
    e << YAML::Key << "type" << YAML::Value << "interval";

    if(values_.first.type() == typeid(int)) {
        e << YAML::Key << "int" << YAML::Value << YAML::BeginSeq;
        e << boost::any_cast<int> (values_.first) << boost::any_cast<int> (values_.second);
        e << YAML::EndSeq;

    } else if(values_.first.type() == typeid(double)) {
        e << YAML::Key << "double" << YAML::Value << YAML::BeginSeq;
        e << boost::any_cast<double> (values_.first) << boost::any_cast<double> (values_.second);
        e << YAML::EndSeq;
    }
}

namespace {
template <typename T>
std::pair<T,T> __read(const YAML::Node& n) {
    std::pair<T,T> v;
    assert(n.Type() == YAML::NodeType::Sequence);
    n[0] >> v.first;
    n[1] >> v.second;
    return v;
}
}

void IntervalParameter::doRead(const YAML::Node& n)
{
    if(!exists(n, "name")) {
        return;
    }

    n["name"] >> name_;

    if(exists(n, "int")) {
        values_ = __read<int>(n["int"]);

    } else if(exists(n, "double")) {
        values_ = __read<double>(n["double"]);

    } else {
        throw std::runtime_error("cannot read interval");
    }
}
