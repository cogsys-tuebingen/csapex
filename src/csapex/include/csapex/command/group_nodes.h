#ifndef GROUP_NODES_H
#define GROUP_NODES_H

/// COMPONENT
#include "group_base.h"

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex
{
namespace command
{

class GroupNodes : public GroupBase
{
public:
    GroupNodes(const AUUID &graph_uuid, const std::vector<UUID>& nodes);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;


protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    void findNodes(Graph* graph);
    void mapConnections(AUUID parent_auuid, AUUID sub_graph_auuid);

    void mapMessageGoingIn(AUUID parent_auuid, AUUID sub_graph_auuid);
    void mapMessageGoingOut(AUUID parent_auuid, AUUID sub_graph_auuid);
    void mapSignalGoingIn(AUUID parent_auuid, AUUID sub_graph_auuid);
    void mapSignalGoingOut(AUUID parent_auuid, AUUID sub_graph_auuid);

private:
    std::vector<UUID> uuids;

    UUID sub_graph_uuid_;
};

}

}
#endif // GROUP_NODES_H
