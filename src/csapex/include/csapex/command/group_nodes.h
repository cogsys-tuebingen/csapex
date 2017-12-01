#ifndef GROUP_NODES_H
#define GROUP_NODES_H

/// COMPONENT
#include "group_base.h"
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT GroupNodes : public GroupBase
{
    COMMAND_HEADER(GroupNodes);

public:
    GroupNodes(const AUUID &graph_uuid, const std::vector<UUID>& nodes);

    virtual std::string getDescription() const override;    

    void serialize(SerializationBuffer &data) const override;
    void deserialize(const SerializationBuffer& data) override;
    void cloneFrom(const Command& other) override;

    std::string getType() const override
    {
        return typeName();
    }
    static std::string typeName()
    {
        return type2nameWithoutNamespace(typeid(GroupNodes));
    }

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    void findNodes(SubgraphNode *graph);
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
