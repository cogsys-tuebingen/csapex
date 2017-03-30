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
    DeleteConnection(const AUUID& graph_uuid, Connectable* a, Connectable* b);

    void serialize(SerializationBuffer &data) const override;
    void deserialize(SerializationBuffer& data) override;

    std::string getType() const override
    {
        return typeName();
    }
    static std::string typeName()
    {
        return type2nameWithoutNamespace(typeid(DeleteConnection));
    }

protected:
    virtual bool doExecute() override;
    virtual bool doRedo() override;
    virtual bool doUndo() override;

    virtual std::string getDescription() const override;

protected:
    int connection_id;
    bool active_;

    UUID from_uuid;
    UUID to_uuid;
};
}
}

#endif // COMMAND_DELETE_CONNECTION_H
