/// HEADER
#include <csapex/param/string_list_parameter.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <boost/any.hpp>

using namespace csapex;
using namespace param;

StringListParameter::StringListParameter()
    : Parameter("noname", ParameterDescription())
{
}

StringListParameter::StringListParameter(const std::string &name, const ParameterDescription &description)
    : Parameter(name, description)
{
}

StringListParameter::~StringListParameter()
{

}


const std::type_info& StringListParameter::type() const
{
    Lock l = lock();
    return typeid(list_);
}

std::string StringListParameter::toStringImpl() const
{
    std::stringstream v;

    for(std::size_t i = 0, n = list_.size(); i < n; ++i) {
        if(i > 0) {
            v << ", ";
        }
        v << list_[i];
    }

    return std::string("[string_list: ") + v.str()  + "]";
}

void StringListParameter::get_unsafe(boost::any& out) const
{
    out = list_;
}


bool StringListParameter::set_unsafe(const boost::any &v)
{
    auto l = boost::any_cast<std::vector<std::string> >(v);
    if(list_ != l) {
        list_ = l;
        return true;
    }

    return false;
}


void StringListParameter::doSetValueFrom(const Parameter &other)
{
    const StringListParameter* list = dynamic_cast<const StringListParameter*>(&other);
    if(list) {
        list_ = list->list_;
        triggerChange();
    } else {
        throw std::runtime_error("bad setFrom, invalid types");
    }
}

void StringListParameter::doClone(const Parameter &other)
{
    const StringListParameter* list = dynamic_cast<const StringListParameter*>(&other);
    if(list) {
        list_ = list->list_;
    } else {
        throw std::runtime_error("bad clone, invalid types");
    }
}

void StringListParameter::doSerialize(YAML::Node& n) const
{
    n["list"] = list_;
}

void StringListParameter::doDeserialize(const YAML::Node& n)
{
    if(n["list"].IsDefined()) {
        list_ = n["list"].as<std::vector<std::string> >();
    }
}


void StringListParameter::add(const std::string& value)
{
    list_.push_back(value);
}

void StringListParameter::setAt(std::size_t i, const std::string &value)
{
    list_.at(i) = value;
}

void StringListParameter::remove(std::size_t i)
{
    list_.erase(list_.begin() + i);
}

void StringListParameter::removeAll(const std::string &value)
{
    list_.erase(std::remove(list_.begin(), list_.end(), value), list_.end());
}

std::size_t StringListParameter::count() const
{
    return list_.size();
}
std::vector<std::string> StringListParameter::getValues() const
{
    return list_;
}
