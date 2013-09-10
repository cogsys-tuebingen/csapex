#ifndef COMMAND_ADD_CONNECTION_HPP
#define COMMAND_ADD_CONNECTION_HPP

/// COMPONENT
#include "command.h"
#include <csapex/csapex_fwd.h>

namespace csapex
{


namespace command
{

struct AddConnection : public Command
{
    AddConnection(const std::string& from_uuid, const std::string& to_uuid);

protected:
    bool execute();
    bool undo();
    bool redo();

    void refresh();

private:
    ConnectorOut* from;
    ConnectorIn* to;

    std::string from_uuid;
    std::string to_uuid;
};
}
}
#endif // COMMAND_ADD_CONNECTION_HPP
