#ifndef COMMAND_MOVE_BOX_H
#define COMMAND_MOVE_BOX_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>
#include <csapex/data/point.h>

namespace csapex
{
namespace command
{
class CSAPEX_COMMAND_EXPORT MoveBox : public CommandImplementation<MoveBox>
{
    COMMAND_HEADER(MoveBox);

public:
    MoveBox(const AUUID& graph_uuid, const UUID& node_uuid, Point from, Point to);

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getDescription() const override;

protected:
    Point from;
    Point to;

    UUID box_uuid;
};

}  // namespace command
}  // namespace csapex

#endif  // COMMAND_MOVE_BOX_H
