#ifndef COMMAND_DELETE_CONNECTION_H
#define COMMAND_DELETE_CONNECTION_H

/// COMPONENT
#include "meta.h"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{
class CSAPEX_COMMAND_EXPORT DeleteConnection : public Meta
{
    COMMAND_HEADER(DeleteConnection);

public:
    DeleteConnection(const AUUID& graph_uuid, const UUID& from, const UUID& to);

    void serialize(SerializationBuffer& data, SemanticVersion& version) const override;
    void deserialize(const SerializationBuffer& data, const SemanticVersion& version) override;

    std::string getType() const override
    {
        return typeName();
    }
    static std::string typeName()
    {
        return type2nameWithoutNamespace(typeid(DeleteConnection));
    }

protected:
    bool doExecute() override;
    bool doRedo() override;
    bool doUndo() override;

    std::string getDescription() const override;

protected:
    int connection_id;
    bool active_;

    UUID from_uuid;
    UUID to_uuid;
};
}  // namespace command
}  // namespace csapex

#endif  // COMMAND_DELETE_CONNECTION_H
