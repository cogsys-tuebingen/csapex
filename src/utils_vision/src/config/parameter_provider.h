#ifndef PARAMETER_PROVIDER_H
#define PARAMETER_PROVIDER_H

/// COMPONENT
#include "parameter.h"

/// SYSTEM
#include <string>
#include <vector>
#include <map>

namespace vision {

class ParameterProvider
{
public:
    ParameterProvider();

    const Parameter operator() (const std::string& name) const;
    Parameter& operator[] (const std::string& name);

    virtual Parameter& getParameter(const std::string& name) = 0;
    virtual const Parameter& getConstParameter(const std::string& name) const = 0;
};

class StaticParameterProvider : public ParameterProvider
{
    typedef std::vector<Parameter> Vec;
    typedef std::map<std::string, Parameter> Map;

public:
    StaticParameterProvider(const Vec& params);
    StaticParameterProvider(const Map& params);

    virtual Parameter& getParameter(const std::string& name);
    virtual const Parameter& getConstParameter(const std::string& name) const;
private:
    Map params_;
};

}

#endif // PARAMETER_PROVIDER_H
