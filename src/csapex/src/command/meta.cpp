/// HEADER
#include <csapex/command/meta.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex::command;

Meta::Meta()
    : locked(false)
{
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
        bool s = Access::executeCommand(graph_, cmd);
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
        bool s = Access::undoCommand(graph_, cmd);
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
        bool s = Access::redoCommand(graph_, cmd);
        success &= s;
    }
    return success;
}
