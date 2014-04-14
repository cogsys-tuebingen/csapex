/// HEADER
#include <utils_param/path_parameter.h>

using namespace param;

PathParameter::PathParameter()
    : Parameter("noname")
{
}

PathParameter::PathParameter(const std::string &name)
    : Parameter(name)
{
}

PathParameter::~PathParameter()
{

}


const std::type_info& PathParameter::type() const
{
    return typeid(std::string);
}

std::string PathParameter::toStringImpl() const
{
    return std::string("[path: ") + value_  + "]";
}

boost::any PathParameter::get_unsafe() const
{
    return value_;
}


void PathParameter::set_unsafe(const boost::any &v)
{
    value_ = boost::any_cast<std::string>(v);
}


void PathParameter::doSetFrom(const Parameter &other)
{
    const PathParameter* range = dynamic_cast<const PathParameter*>(&other);
    if(range) {
        value_ = range->value_;
        triggerChange();
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void PathParameter::doWrite(YAML::Emitter& e) const
{
    e << YAML::Key << "type" << YAML::Value << "path";
    e << YAML::Key << "value" << YAML::Value <<value_;
}

namespace {
template <typename T>
T __read(const YAML::Node& n) {
    T v;
    n >> v;
    return v;
}
}

void PathParameter::doRead(const YAML::Node& n)
{
    if(!exists(n, "name")) {
        return;
    }

    n["name"] >> name_;
    n["value"] >> value_;
}
