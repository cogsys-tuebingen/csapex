/// HEADER
#include <utils_param/set_parameter.h>

using namespace param;

SetParameter::SetParameter()
    : Parameter("noname")
{
}


SetParameter::SetParameter(const std::string &name)
    : Parameter(name)
{
}

SetParameter::~SetParameter()
{

}

void SetParameter::setByName(const std::string &name)
{
    std::map<std::string, variant>::iterator pos = set_.find(name);
    if(pos == set_.end()) {
        throw std::runtime_error(std::string("no such parameter: ") + name);
    }

    value_ = pos->second;
    triggerChange();
}

int SetParameter::noParameters() const
{
    return set_.size();
}

std::string SetParameter::convertToString(const variant &v) const
{
    if(v.type() == typeid(std::string)) {
        return boost::any_cast<std::string> (v);
    }

    std::stringstream ss;
    if(v.type() == typeid(int)) {
        ss << boost::any_cast<int> (v);

    } else if(v.type() == typeid(double)) {
        ss << boost::any_cast<double> (v);

    } else if(v.type() == typeid(bool)) {
        ss << boost::any_cast<bool> (v);

    } else {
        throw std::runtime_error("unsupported type");
    }

    return ss.str();
}

std::string SetParameter::getName(int idx) const
{
    std::map<std::string, variant>::const_iterator i = set_.begin();
    std::advance(i, idx);
    return i->first;
}

std::string SetParameter::getName() const
{
    std::string str =  convertToString(value_);
    for(std::map<std::string, variant>::const_iterator it = set_.begin(); it != set_.end(); ++it) {
        if(convertToString(it->second) == str) {
            return it->first;
        }
    }

    throw std::runtime_error("cannot get the name for parameter '" + name() + "'");
}


const std::type_info& SetParameter::type() const
{
    return value_.type();
}

std::string SetParameter::toStringImpl() const
{
    return std::string("[set: ") + convertToString(value_)  + "]";
}

boost::any SetParameter::get_unsafe() const
{
    return value_;
}


void SetParameter::set_unsafe(const boost::any &v)
{
    value_ = v;
}


void SetParameter::doSetFrom(const Parameter &other)
{
    const SetParameter* range = dynamic_cast<const SetParameter*>(&other);
    if(range) {
        value_ = range->value_;
        triggerChange();
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void SetParameter::doWrite(YAML::Emitter& e) const
{
    e << YAML::Key << "type" << YAML::Value << "set";

    if(value_.type() == typeid(int)) {
        e << YAML::Key << "int" << YAML::Value << boost::any_cast<int> (value_);

    } else if(value_.type() == typeid(double)) {
        e << YAML::Key << "double" << YAML::Value << boost::any_cast<double> (value_);

    } else if(value_.type() == typeid(bool)) {
        e << YAML::Key << "bool" << YAML::Value << boost::any_cast<bool> (value_);

    } else if(value_.type() == typeid(std::string)) {
        e << YAML::Key << "string" << YAML::Value << boost::any_cast<std::string> (value_);
    }
}

namespace {
template <typename T>
T __read(const YAML::Node& n) {
    T v;
    n >> v;
    return v;
}
}

void SetParameter::doRead(const YAML::Node& n)
{
    if(!n.FindValue("name")) {
        return;
    }

    n["name"] >> name_;

    if(n.FindValue("int")) {
        value_ = __read<int>(n["int"]);

    } else if(n.FindValue("double")) {
        value_ = __read<double>(n["double"]);

    } else if(n.FindValue("bool")) {
        value_ = __read<bool>(n["bool"]);

    } else if(n.FindValue("string")) {
        value_ = __read<std::string>(n["string"]);
    }
}
