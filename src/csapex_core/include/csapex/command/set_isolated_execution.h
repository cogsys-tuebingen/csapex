#ifndef SET_ISOLATED_EXECUTION_H
#define SET_ISOLATED_EXECUTION_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>
#include <csapex/model/execution_type.h>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT SetIsolatedExecution : public CommandImplementation<SetIsolatedExecution>
{
    COMMAND_HEADER(SetIsolatedExecution);

public:
    SetIsolatedExecution(const AUUID &graph_uuid, const UUID& node, ExecutionType type);

    virtual std::string getDescription() const override;

    void serialize(SerializationBuffer &data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    ExecutionType type_;
    ExecutionType was_type_;
};

}

}
#endif // SET_ISOLATED_EXECUTION_H

