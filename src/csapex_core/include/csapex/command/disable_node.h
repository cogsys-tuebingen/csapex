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
    DisableNode(const AUUID& graph_uuid, const UUID& uuid, bool disable = true);

public:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    std::string getDescription() const override;

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    UUID uuid;
    bool disable_;
};

}  // namespace command
}  // namespace csapex
#endif  // COMMAND_DISABLE_NODE_H
