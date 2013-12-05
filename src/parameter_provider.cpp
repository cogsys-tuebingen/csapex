/// HEADER
#include <utils_param/parameter_provider.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace param;

ParameterProvider::ParameterProvider()
{
}

const Parameter::Ptr ParameterProvider::operator () (const std::string& name) const
{
    return Parameter::Ptr(getConstParameter(name));
}

Parameter& ParameterProvider::operator [] (const std::string& name)
{
    return *getParameter(name);
}

StaticParameterProvider::StaticParameterProvider(const Map& params)
    : params_(params)
{

}
StaticParameterProvider::StaticParameterProvider(const Vec& params)
{
    BOOST_FOREACH(const Parameter::Ptr param, params) {
        params_.insert(std::make_pair(param->name(), param));
    }
}


Parameter::Ptr StaticParameterProvider::getParameter(const std::string& name)
{
    return params_.at(name);
}

const Parameter::Ptr StaticParameterProvider::getConstParameter(const std::string& name) const
{
    return params_.at(name);
}
