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
    COMMAND_HEADER(DeleteNode);

public:
    DeleteNode(const AUUID &graph_uuid, const UUID &uuid);

    void serialize(SerializationBuffer &data) const override;
    void deserialize(const SerializationBuffer& data) override;


    std::string getType() const override
    {
        return typeName();
    }
    static std::string typeName()
    {
        return type2nameWithoutNamespace(typeid(DeleteNode));
    }

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

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
