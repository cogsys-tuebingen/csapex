#ifndef COMMAND_ADD_NODE_HPP
#define COMMAND_ADD_NODE_HPP

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>
#include <csapex/data/point.h>

namespace csapex
{
namespace command
{
class CSAPEX_COMMAND_EXPORT AddNode : public CommandImplementation<AddNode>
{
    COMMAND_HEADER(AddNode);

public:
    AddNode(const AUUID& graph_uuid, const std::string& type, Point pos, const UUID& uuid_, NodeStatePtr state);

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getDescription() const override;

private:
    std::string type_;
    Point pos_;

    UUID uuid_;

    NodeStatePtr saved_state_;
};
}  // namespace command
}  // namespace csapex

#endif  // COMMAND_ADD_NODE_HPP
