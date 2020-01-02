#ifndef MINIMIZE_H
#define MINIMIZE_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{
class CSAPEX_COMMAND_EXPORT Minimize : public CommandImplementation<Minimize>
{
    COMMAND_HEADER(Minimize);

public:
    Minimize(const AUUID& graph_uuid, const UUID& node, bool mini);

    std::string getDescription() const override;

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    bool mini;
    bool executed;
};

}  // namespace command

}  // namespace csapex
#endif  // MINIMIZE_H
