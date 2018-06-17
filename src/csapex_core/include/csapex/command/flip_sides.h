#ifndef FLIP_SIDES_H
#define FLIP_SIDES_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct CSAPEX_COMMAND_EXPORT FlipSides : public CommandImplementation<FlipSides>
{
    COMMAND_HEADER(FlipSides);

public:
    FlipSides(const AUUID &graph_uuid, const UUID& node);

    virtual std::string getDescription() const override;

    void serialize(SerializationBuffer &data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
};

}

}
#endif // FLIP_SIDES_H

