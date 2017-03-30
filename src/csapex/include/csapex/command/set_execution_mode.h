#ifndef SET_EXECUTION_MODE_H
#define SET_EXECUTION_MODE_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>
#include <csapex/model/execution_mode.h>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT SetExecutionMode : public CommandImplementation<SetExecutionMode>
{
    COMMAND_HEADER(SetExecutionMode);

public:
    SetExecutionMode(const AUUID &graph_uuid, const UUID& node, ExecutionMode mode);

    virtual std::string getDescription() const override;

    void serialize(SerializationBuffer &data) const override;
    void deserialize(SerializationBuffer& data) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    ExecutionMode was_mode;
    ExecutionMode mode;
};

}

}
#endif // SET_EXECUTION_MODE_H

