#ifndef PARAMETER_PROVIDER_H
#define PARAMETER_PROVIDER_H

/// COMPONENT
#include "parameter.h"

/// SYSTEM
#include <string>
#include <vector>
#include <map>

namespace csapex {
namespace param {

class ParameterProvider
{
public:
    ParameterProvider();

    const Parameter::Ptr operator() (const std::string& name) const;
    Parameter& operator[] (const std::string& name);

    virtual Parameter::Ptr getParameter(const std::string& name) = 0;
    virtual const Parameter::Ptr getConstParameter(const std::string& name) const = 0;
};

class StaticParameterProvider : public ParameterProvider
{
    typedef std::vector<Parameter::Ptr> Vec;
    typedef std::map<std::string, Parameter::Ptr> Map;

public:
    StaticParameterProvider(const Vec& params);
    StaticParameterProvider(const Map& params);

    virtual Parameter::Ptr getParameter(const std::string& name) override;
    virtual const Parameter::Ptr getConstParameter(const std::string& name) const override;
private:
    Map params_;
};

}
}

#endif // PARAMETER_PROVIDER_H
