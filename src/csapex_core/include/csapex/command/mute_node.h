#ifndef MUTE_NODE_H
#define MUTE_NODE_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT MuteNode : public CommandImplementation<MuteNode>
{
    COMMAND_HEADER(MuteNode);

public:
    MuteNode(const AUUID &graph_uuid, const UUID& node, bool muted);

    virtual std::string getDescription() const override;

    void serialize(SerializationBuffer &data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    bool muted;
    bool executed;
};

}

}
#endif // MUTE_NODE_H

