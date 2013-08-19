#ifndef COMMAND_ADD_CONNECTOR_H
#define COMMAND_ADD_CONNECTOR_H

/// COMPONENT
#include "command.h"

namespace csapex
{
class Box;
class Connector;

namespace command
{

struct AddConnector : public Command
{
    AddConnector(const std::string& box_uuid, bool input, const std::string& uuid, bool forward = false);

protected:
    bool execute();
    bool undo();
    bool redo();

    void refresh();

private:
    Box* box;
    bool input;

    Connector* c;

    std::string b_uuid;
    std::string c_uuid;

    bool forward;
};

}

}

#endif // COMMAND_ADD_CONNECTOR_H
