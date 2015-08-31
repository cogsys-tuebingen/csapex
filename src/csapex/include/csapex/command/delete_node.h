#ifndef COMMAND_DELETE_NODE_H
#define COMMAND_DELETE_NODE_H

/// COMPONENT
#include <csapex/command/meta.h>
#include <csapex/utility/uuid.h>

namespace csapex
{

namespace command
{
class DeleteNode : public Meta
{
public:
    DeleteNode(const UUID &uuid);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    std::string type;
    UUID uuid;

//    Command::Ptr remove_connections;

    NodeStatePtr saved_state;
};

}
}
#endif // COMMAND_DELETE_NODE_H
