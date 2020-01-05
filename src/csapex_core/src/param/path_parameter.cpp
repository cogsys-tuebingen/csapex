/// HEADER
#include <csapex/param/path_parameter.h>

/// PROJECT
#include <csapex/param/register_parameter.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/utility/yaml.h>

/// SYSTEM
#include <boost/any.hpp>

CSAPEX_REGISTER_PARAM(PathParameter)

using namespace csapex;
using namespace param;

PathParameter::PathParameter() : ParameterImplementation("noname", ParameterDescription())
{
}

PathParameter::PathParameter(const std::string& name, const ParameterDescription& description, const std::string& filter, bool is_file, bool input, bool output)
  : ParameterImplementation(name, description), filter_(filter), is_file_(is_file), input_(input), output_(output)
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
    return std::string("[path: ") + value_ + "]";
}

void PathParameter::get_unsafe(boost::any& out) const
{
    out = value_;
}

bool PathParameter::set_unsafe(const boost::any& v)
{
    auto val = boost::any_cast<std::string>(v);
    if (value_ != val) {
        value_ = val;
        return true;
    }

    return false;
}

bool PathParameter::cloneDataFrom(const Clonable& other)
{
    if (const PathParameter* path = dynamic_cast<const PathParameter*>(&other)) {
        bool value_changed = value_ != path->value_;
        *this = *path;
        if (value_changed) {
            triggerChange();
            return true;
        }
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }

    return false;
}

void PathParameter::doSerialize(YAML::Node& n) const
{
    n["version"] = 2;

    n["value"] = value_;
    n["def"] = def_;

    n["filter"] = filter_;
    n["is_file"] = is_file_;
    n["is_input"] = input_;
    n["is_output"] = output_;
}

void PathParameter::doDeserialize(const YAML::Node& n)
{
    int version = n["version"].IsDefined() ? n["version"].as<int>() : 1;

    value_ = n["value"].as<std::string>();

    if (version > 1) {
        def_ = n["def"].as<std::string>();

        filter_ = n["filter"].as<std::string>();
        is_file_ = n["is_file"].as<bool>();
        input_ = n["is_input"].as<bool>();
        output_ = n["is_output"].as<bool>();
    }
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

void PathParameter::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    Parameter::serialize(data, version);

    data << value_;
    data << def_;

    data << filter_;
    data << is_file_;
    data << input_;
    data << output_;
}

void PathParameter::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Parameter::deserialize(data, version);

    data >> value_;
    data >> def_;

    data >> filter_;
    data >> is_file_;
    data >> input_;
    data >> output_;
}
