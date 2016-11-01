#ifndef GROUP_BASE_H
#define GROUP_BASE_H

/// COMPONENT
#include "meta.h"
#include <csapex/data/point.h>
#include <csapex/utility/uuid.h>
#include <csapex/model/connection_information.h>
#include <csapex/serialization/snippet.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <unordered_map>

namespace csapex
{
namespace command
{
class CSAPEX_COMMAND_EXPORT GroupBase : public Meta
{
protected:
    GroupBase(const AUUID &graph_uuid, const std::string &type);

    void setNodes(const std::vector<NodeHandle*>& nodes);
    csapex::Point findTopLeftPoint() const;

    void analyzeConnections(Graph* graph);
    void pasteSelection(AUUID sub_graph_auuid);

protected:

    std::set<NodeHandle*> node_set;
    std::vector<NodeHandle*> nodes;

    Point insert_pos;

    std::vector<ConnectionInformation> connections_going_in;
    std::vector<ConnectionInformation> connections_going_out;
    std::vector<ConnectionInformation> signals_going_in;
    std::vector<ConnectionInformation> signals_going_out;


    Snippet serialized_snippet_;
    std::unordered_map<UUID, UUID, UUID::Hasher> old_uuid_to_new;
};

}
}

#endif // GROUP_BASE_H
