#ifndef MODIFY_CONNECTION_H
#define MODIFY_CONNECTION_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/data/point.h>

namespace csapex
{
namespace command
{
class CSAPEX_COMMAND_EXPORT ModifyConnection : public CommandImplementation<ModifyConnection>
{
    COMMAND_HEADER(ModifyConnection);

public:
    ModifyConnection(const AUUID& graph_uuid, int connection_id, bool active);

    virtual std::string getDescription() const override;

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    int connection_id;

    bool was_active;
    bool active;
};

}  // namespace command
}  // namespace csapex

#endif  // MODIFY_CONNECTION_H
