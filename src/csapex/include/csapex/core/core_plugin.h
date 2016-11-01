#ifndef CORE_PLUGIN_H
#define CORE_PLUGIN_H

/// PROJECT
#include <csapex/core/csapex_core.h>
#include <csapex/utility/export_plugin.h>

/// SYSTEM
#include <memory>

namespace csapex {
class CSAPEX_EXPORT CorePlugin {
public:
    typedef std::shared_ptr<CorePlugin> Ptr;

public:
    virtual ~CorePlugin();
    virtual void prepare(Settings& settings);
    virtual void init(CsApexCore& core) = 0;
    virtual void setupGraph(SubgraphNode* graph);
    virtual void shutdown();

    void setName(const std::string& name);
    std::string getName();

private:
    std::string name_;
};
}

#endif // CORE_PLUGIN_H
