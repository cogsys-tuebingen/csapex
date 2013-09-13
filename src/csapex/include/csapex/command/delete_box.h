#ifndef COMMAND_DELETE_BOX_H
#define COMMAND_DELETE_BOX_H

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/memento.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QPoint>
#include <QWidget>

namespace csapex
{

namespace command
{
class DeleteBox : public Command
{
public:
    DeleteBox(const std::string &uuid);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

protected:
    QWidget* parent;
    QPoint pos;

    std::string type;
    std::string uuid;

    Command::Ptr remove_connections;

    Memento::Ptr saved_state;
};

}
}
#endif // COMMAND_DELETE_BOX_H
