#ifndef COMMAND_ADD_CONNECTOR_H
#define COMMAND_ADD_CONNECTOR_H

/// COMPONENT
#include "command.h"
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

class AddConnector : public Command
{
public:
    AddConnector(const UUID &box_uuid, const std::string& label, const std::string& type, bool input, const UUID &uuid);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    virtual std::string getType() const;
    virtual std::string getDescription() const;

private:
    std::string type;
    std::string label;
    bool input;


    Connectable* c;

    UUID b_uuid;
    UUID c_uuid;
};

}

}

#endif // COMMAND_ADD_CONNECTOR_H
