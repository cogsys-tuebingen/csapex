#ifndef GROUP_BASE_H
#define GROUP_BASE_H

/// COMPONENT
#include "meta.h"
#include <csapex/data/point.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <unordered_map>

namespace csapex
{
namespace command
{
class GroupBase : public Meta
{
protected:
    GroupBase(const AUUID &graph_uuid, const std::string &type);

    void setNodes(const std::vector<NodeHandle*>& nodes);
    csapex::Point findTopLeftPoint() const;

    void analyzeConnections(Graph* graph);
    void pasteSelection(AUUID sub_graph_auuid);

protected:
    struct ConnectionInformation {
        UUID from;
        UUID to;
        TokenConstPtr type;
    };

    std::set<NodeHandle*> node_set;
    std::vector<NodeHandle*> nodes;

    Point insert_pos;

    std::vector<ConnectionInformation> connections_going_in;
    std::vector<ConnectionInformation> connections_going_out;
    std::vector<ConnectionInformation> signals_going_in;
    std::vector<ConnectionInformation> signals_going_out;


    YAML::Node selection_yaml;
    std::unordered_map<UUID, UUID, UUID::Hasher> old_uuid_to_new;
};

}
}

#endif // GROUP_BASE_H
