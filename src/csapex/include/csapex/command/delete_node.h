#ifndef COMMAND_DELETE_NODE_H
#define COMMAND_DELETE_NODE_H

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QPoint>
#include <QWidget>

namespace csapex
{

namespace command
{
class DeleteNode : public Command
{
public:
    DeleteNode(const std::string &uuid);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

protected:
    QPoint pos;

    std::string type;
    std::string uuid;

    Command::Ptr remove_connections;

    NodeStatePtr saved_state;
};

}
}
#endif // COMMAND_DELETE_NODE_H
