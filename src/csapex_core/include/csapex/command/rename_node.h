#ifndef RENAME_NODE_H
#define RENAME_NODE_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{
class CSAPEX_COMMAND_EXPORT RenameNode : public CommandImplementation<RenameNode>
{
    COMMAND_HEADER(RenameNode);

public:
    RenameNode(const AUUID& graph_uuid, const UUID& node, const std::string& new_name);

    virtual std::string getDescription() const override;

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    std::string new_name_;
    std::string old_name_;
};

}  // namespace command

}  // namespace csapex
#endif  // RENAME_NODE_H
