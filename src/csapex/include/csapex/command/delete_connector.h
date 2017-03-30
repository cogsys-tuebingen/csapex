#ifndef COMMAND_DELETE_CONNECTOR_H
#define COMMAND_DELETE_CONNECTOR_H

/// COMPONENT
#include "command_impl.hpp"
#include <csapex/utility/uuid.h>

namespace csapex
{

namespace command
{

class CSAPEX_COMMAND_EXPORT DeleteConnector : public CommandImplementation<DeleteConnector>
{
    COMMAND_HEADER(DeleteConnector);

public:
    DeleteConnector(const AUUID &graph_uuid, Connectable *_c);

    void serialize(SerializationBuffer &data) const override;
    void deserialize(SerializationBuffer& data) override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getDescription() const override;

private:
    bool       in;

    Command::Ptr delete_connections;

    UUID c_uuid;

};
}
}
#endif // COMMAND_DELETE_CONNECTOR_H
