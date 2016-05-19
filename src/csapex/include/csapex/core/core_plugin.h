#ifndef CORE_PLUGIN_H
#define CORE_PLUGIN_H

/// PROJECT
#include <csapex/core/csapex_core.h>

/// SYSTEM
#include <memory>

namespace csapex {
class CorePlugin {
public:
    typedef std::shared_ptr<CorePlugin> Ptr;

public:
    virtual ~CorePlugin();
    virtual void prepare(Settings& settings);
    virtual void init(CsApexCore& core) = 0;
    virtual void setupGraph(Graph* graph);
    virtual void shutdown();

    void setName(const std::string& name);
    std::string getName();

private:
    std::string name_;
};
}

#endif // CORE_PLUGIN_H
