/// HEADER
#include <csapex/param/path_parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <boost/any.hpp>

using namespace csapex;
using namespace param;

PathParameter::PathParameter()
    : Parameter("noname", ParameterDescription())
{
}

PathParameter::PathParameter(const std::string &name, const ParameterDescription& description, const std::string &filter, bool is_file, bool input, bool output)
    : Parameter(name, description), filter_(filter), is_file_(is_file), input_(input), output_(output)
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

void PathParameter::get_unsafe(boost::any& out) const
{
    out = value_;
}


bool PathParameter::set_unsafe(const boost::any &v)
{
    auto val = boost::any_cast<std::string>(v);
    if(value_ != val) {
        value_ = val;
        return true;
    }

    return false;
}


void PathParameter::doSetValueFrom(const Parameter &other)
{
    const PathParameter* path = dynamic_cast<const PathParameter*>(&other);
    if(path) {
        if(value_ != path->value_) {
            value_ = path->value_;
            triggerChange();
        }
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void PathParameter::doClone(const Parameter &other)
{
    const PathParameter* path = dynamic_cast<const PathParameter*>(&other);
    if(path) {
        value_ = path->value_;
        def_ = path->def_;
    } else {
        throw std::runtime_error("bad clone, invalid types");
    }
}

void PathParameter::doSerialize(YAML::Node& n) const
{
    n["value"] = value_;
}

void PathParameter::doDeserialize(const YAML::Node& n)
{
    value_ = n["value"].as<std::string>();
}

std::string PathParameter::def() const
{
    return def_;
}

std::string PathParameter::filter() const
{
    return filter_;
}

bool PathParameter::isFile() const
{
    return is_file_;
}

bool PathParameter::isInput() const
{
    return input_;
}

bool PathParameter::isOutput() const
{
    return output_;
}
