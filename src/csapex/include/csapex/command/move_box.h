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
    MoveBox(const AUUID &graph_uuid, const UUID& node_uuid, Point from, Point to);

    void serialize(SerializationBuffer &data) const override;
    void deserialize(SerializationBuffer& data) override;

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    virtual std::string getDescription() const;

protected:
    Point from;
    Point to;

    UUID box_uuid;
};

}
}

#endif // COMMAND_MOVE_BOX_H
