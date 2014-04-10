/// HEADER
#include <csapex/command/meta.h>

/// SYSTEM
#include <boost/foreach.hpp>

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

    BOOST_FOREACH(Command::Ptr cmd, nested) {
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


void Meta::add(Command::Ptr cmd)
{
    assert(!locked);
    assert(cmd);
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
    BOOST_FOREACH(Command::Ptr cmd, nested) {
        bool s = Access::executeCommand(graph_, widget_ctrl_, cmd);
        if(!s) {
            std::cerr << "command failed to execute! (" << typeid(*cmd).name() << ")" << std::endl;
        }
        success &= s;
    }
    return success;
}

bool Meta::doUndo()
{
    BOOST_REVERSE_FOREACH(Command::Ptr cmd, nested) {
        bool s = Access::undoCommand(graph_, widget_ctrl_, cmd);
        if(!s) {
            undo_later.push_back(cmd);
        }
    }

    return true;
}

bool Meta::doRedo()
{
    bool success = true;
    BOOST_FOREACH(Command::Ptr cmd, nested) {
        bool s = Access::redoCommand(graph_, widget_ctrl_, cmd);
        success &= s;
    }
    return success;
}
