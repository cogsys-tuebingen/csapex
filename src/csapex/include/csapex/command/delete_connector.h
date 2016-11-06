#ifndef COMMAND_DELETE_CONNECTOR_H
#define COMMAND_DELETE_CONNECTOR_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>

namespace csapex
{

namespace command
{

struct CSAPEX_COMMAND_EXPORT DeleteConnector : public Command
{
    DeleteConnector(const AUUID &graph_uuid, Connectable *_c);

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    bool refresh();

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

private:
    bool       in;
    ConnectablePtr c;

    Command::Ptr delete_connections;

    UUID c_uuid;

};
}
}
#endif // COMMAND_DELETE_CONNECTOR_H
