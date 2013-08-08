#ifndef COMMAND_ADD_BOX_HPP
#define COMMAND_ADD_BOX_HPP

/// COMPONENT
#include "command.h"
#include "memento.h"

/// SYSTEM
#include <QWidget>

namespace csapex
{

class SelectorProxy;
class Box;

namespace command
{

struct AddBox : public Command
{
    AddBox(SelectorProxy* selector, QWidget* parent, QPoint pos);

protected:
    bool execute(Graph& graph);
    bool undo(Graph& graph);
    bool redo(Graph& graph);

    void refresh(Graph& graph);

private:
    SelectorProxy* selector;
    QWidget* parent;
    QPoint pos;

    std::string type;
    std::string uuid;

    csapex::Box* box;

    Memento::Ptr saved_state;
};
}
}

#endif // COMMAND_ADD_BOX_HPP
