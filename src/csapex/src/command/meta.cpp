/// HEADER
#include <csapex/command/meta.h>

/// PROJECT
#include <csapex/utility/assert.h>

/// SYSTEM

#include <QTreeWidgetItem>
#include <iostream>

using namespace csapex::command;

Meta::Meta(const std::string &type)
    : locked(false), type(type)
{
}

QTreeWidgetItem* Meta::createDebugInformation() const
{
    QTreeWidgetItem* tl = new QTreeWidgetItem;
    tl->setText(0, getType().c_str());
    tl->setText(1, getDescription().c_str());

    for(Command::Ptr cmd : nested) {
        tl->addChild(cmd->createDebugInformation());
    }
    return tl;
}

std::string Meta::getType() const
{
    return type;
}

std::string Meta::getDescription() const
{
    return "";
}

void Meta::clear()
{
    apex_assert_hard(!locked);
    nested.clear();
}

void Meta::add(Command::Ptr cmd)
{
    apex_assert_hard(!locked);
    apex_assert_hard(cmd);
    nested.push_back(cmd);
}

int Meta::commands() const
{
    return nested.size();
}

bool Meta::doExecute()
{
    locked = true;

    bool success = true;
    for(Command::Ptr cmd : nested) {
        bool s = Access::executeCommand(graph_, node_factory_, cmd);
        if(!s) {
            std::cerr << "command failed to execute! (" << typeid(*cmd).name() << ")" << std::endl;
        }
        success &= s;
    }
    return success;
}

bool Meta::doUndo()
{
    for(auto it = nested.rbegin(); it != nested.rend(); ++it) {
        bool s = Access::undoCommand(graph_, node_factory_, *it);
        if(!s) {
            undo_later.push_back(*it);
        }
    }

    return true;
}

bool Meta::doRedo()
{
    bool success = true;
    for(Command::Ptr cmd : nested) {
        bool s = Access::redoCommand(graph_, node_factory_, cmd);
        success &= s;
    }
    return success;
}
