#ifndef UNGROUP_NODES_H
#define UNGROUP_NODES_H

/// COMPONENT
#include "group_base.h"

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT UngroupNodes : public GroupBase
{
public:
    UngroupNodes(const AUUID &graph_uuid, const UUID &uuid);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    void unmapConnections(AUUID parent_auuid, AUUID sub_graph_auuid);

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;

    SubgraphNodePtr subgraph;

    std::unordered_map<UUID, UUID, UUID::Hasher> old_connections_in;
    std::unordered_map<UUID, std::vector<UUID>, UUID::Hasher> old_connections_out;

    std::unordered_map<UUID, std::vector<UUID>, UUID::Hasher> old_signals_in;
    std::unordered_map<UUID, std::vector<UUID>, UUID::Hasher> old_signals_out;
};

}

}

#endif // UNGROUP_NODES_H
