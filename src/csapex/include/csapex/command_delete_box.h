#ifndef COMMAND_DELETE_BOX_H
#define COMMAND_DELETE_BOX_H

/// COMPONENT
#include "command.h"
#include "memento.h"

/// SYSTEM
#include <QPoint>
#include <QWidget>

namespace csapex
{

class Box;

namespace command
{
class DeleteBox : public Command
{
public:
    DeleteBox(Box* box);

protected:
    bool execute(Graph& graph);
    bool undo(Graph& graph);
    bool redo(Graph& graph);

    void refresh(Graph &graph);

protected:
    Box* box;
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
