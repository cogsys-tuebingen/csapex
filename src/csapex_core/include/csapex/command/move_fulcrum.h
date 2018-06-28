#ifndef COMMAND_MOVE_FULCRUM_H
#define COMMAND_MOVE_FULCRUM_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/data/point.h>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT MoveFulcrum : public CommandImplementation<MoveFulcrum>
{
    COMMAND_HEADER(MoveFulcrum);

public:
    MoveFulcrum(const AUUID& graph_uuid, int connection_id, int fulcrum_id, const Point& from, const Point& to);

    virtual std::string getDescription() const override;

    void serialize(SerializationBuffer &data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    int connection_id;
    int fulcrum_id;
    Point from;
    Point to;
};

}

}

#endif // COMMAND_MOVE_FULCRUM_H
