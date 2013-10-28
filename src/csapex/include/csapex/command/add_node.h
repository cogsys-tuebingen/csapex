#ifndef COMMAND_ADD_NODE_HPP
#define COMMAND_ADD_NODE_HPP

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QWidget>

namespace csapex
{

namespace command
{

struct AddNode : public Command
{
    AddNode(const std::string& type, QPoint pos_, const std::string& parent_uuid_, const std::string& uuid_, NodeStatePtr state);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

private:
    std::string type_;
    QPoint pos_;

    std::string parent_uuid_;
    std::string uuid_;

    NodeStatePtr saved_state_;
};
}
}

#endif // COMMAND_ADD_NODE_HPP
