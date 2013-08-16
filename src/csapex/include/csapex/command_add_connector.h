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
    AddConnector(csapex::Box *box, bool input, const std::string& uuid, bool forward = false);

protected:
    bool execute();
    bool undo();
    bool redo();

private:
    Box* box;
    bool input;

    Connector* c;

    Graph* graph;

    std::string b_uuid;
    std::string c_uuid;

    bool forward;
};

}

}

#endif // COMMAND_ADD_CONNECTOR_H
