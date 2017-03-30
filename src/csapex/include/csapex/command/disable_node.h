#ifndef COMMAND_DISABLE_NODE_H
#define COMMAND_DISABLE_NODE_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>

namespace csapex
{

namespace command
{
class CSAPEX_COMMAND_EXPORT DisableNode : public CommandImplementation<DisableNode>
{
    COMMAND_HEADER(DisableNode);

public:
    DisableNode(const AUUID &graph_uuid, const UUID &uuid, bool disable = true);

public:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getDescription() const override;

    void serialize(SerializationBuffer &data) const override;
    void deserialize(SerializationBuffer& data) override;

protected:
    UUID uuid;
    bool disable_;
};

}
}
#endif // COMMAND_DISABLE_NODE_H
