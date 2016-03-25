#ifndef UNGROUP_NODES_H
#define UNGROUP_NODES_H

/// COMPONENT
#include "meta.h"
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex
{
namespace command
{

struct UngroupNodes : public Meta
{
    UngroupNodes(const AUUID &graph_uuid, const UUID &uuid);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;

    UUID sub_graph_uuid_;

    YAML::Node selection_yaml;
};

}

}

#endif // UNGROUP_NODES_H
