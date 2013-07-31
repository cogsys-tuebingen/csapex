#ifndef CORE_PLUGIN_H
#define CORE_PLUGIN_H

/// SYSTEM
#include <boost/shared_ptr.hpp>

namespace csapex {
class CorePlugin {
public:
    typedef boost::shared_ptr<CorePlugin> Ptr;

public:
    virtual void init() = 0;

    void setName(const std::string& name);
    std::string getName();

private:
    std::string name_;
};
}

#endif // CORE_PLUGIN_H
