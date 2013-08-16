#ifndef COMMAND_ADD_BOX_HPP
#define COMMAND_ADD_BOX_HPP

/// COMPONENT
#include <csapex/command.h>
#include <csapex/memento.h>
#include <csapex/selector_proxy.h>

/// SYSTEM
#include <QWidget>

namespace csapex
{
class Box;
class Graph;

namespace command
{

struct AddBox : public Command
{
    AddBox(Graph& graph, SelectorProxy::Ptr selector, QPoint pos, Memento::Ptr state = Memento::NullPtr, const std::string& uuid = "");

protected:
    bool execute();
    bool undo();
    bool redo();

    void refresh();

private:
    Graph& graph_;
    SelectorProxy::Ptr selector;
    QPoint pos;

    std::string type;
    std::string uuid;

    csapex::Box* box;

    Memento::Ptr saved_state;
};
}
}

#endif // COMMAND_ADD_BOX_HPP
