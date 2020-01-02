#ifndef SET_MAX_EXECUTION_FREQUENCY_H
#define SET_MAX_EXECUTION_FREQUENCY_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>
#include <csapex/model/execution_mode.h>

namespace csapex
{
namespace command
{
class CSAPEX_COMMAND_EXPORT SetMaximumExecutionFrequency : public CommandImplementation<SetMaximumExecutionFrequency>
{
    COMMAND_HEADER(SetMaximumExecutionFrequency);

public:
    SetMaximumExecutionFrequency(const AUUID& graph_uuid, const UUID& node, double frequency);

    std::string getDescription() const override;

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    double was_frequency;
    double frequency;
};

}  // namespace command

}  // namespace csapex

#endif  // SET_MAX_EXECUTION_FREQUENCY_H
