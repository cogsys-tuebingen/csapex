/// HEADER
#include <csapex/command_meta.h>

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
    nested.push_back(cmd);
}

bool Meta::execute()
{
    locked = true;

    bool change = false;
    BOOST_FOREACH(Command::Ptr cmd, nested) {
        change |= cmd->execute();
    }
    return change;
}

bool Meta::undo()
{
    BOOST_REVERSE_FOREACH(Command::Ptr cmd, nested) {
        if(!cmd->undo()) {
//            undo_later.push_back(cmd);
        }
    }

    return true;
}

bool Meta::redo()
{
    bool change = false;
    BOOST_FOREACH(Command::Ptr cmd, nested) {
        change |= cmd->redo();
    }
    return change;
}
