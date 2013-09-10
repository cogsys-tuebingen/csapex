#ifndef COMMAND_ADD_BOX_HPP
#define COMMAND_ADD_BOX_HPP

/// COMPONENT
#include <csapex/command.h>
#include <csapex/memento.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QWidget>

namespace csapex
{

namespace command
{

struct AddBox : public Command
{
    AddBox(const std::string& type, QPoint pos_, const std::string& parent_uuid_, const std::string& uuid_, Memento::Ptr state = Memento::NullPtr);

protected:
    bool execute();
    bool undo();
    bool redo();

private:
    std::string type_;
    QPoint pos_;

    std::string parent_uuid_;
    std::string uuid_;

    Memento::Ptr saved_state_;
};
}
}

#endif // COMMAND_ADD_BOX_HPP
