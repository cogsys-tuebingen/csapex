#ifndef COMMAND_DELETE_NODE_H
#define COMMAND_DELETE_NODE_H

/// COMPONENT
#include <csapex/command/meta.h>
#include <csapex/utility/uuid.h>
#include <csapex/serialization/snippet.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>

namespace csapex
{

namespace command
{
class CSAPEX_COMMAND_EXPORT DeleteNode : public Meta
{
public:
    DeleteNode(const AUUID &graph_uuid, const UUID &uuid);

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    std::string type;
    UUID uuid;

    NodeStatePtr saved_state;
    Snippet saved_graph;
};

}
}
#endif // COMMAND_DELETE_NODE_H
