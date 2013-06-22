/// HEADER
#include "command_meta.h"

/// SYSTEM
#include <boost/foreach.hpp>

using namespace vision_evaluator::command;

Meta::Meta()
    : locked(false)
{
}

void Meta::add(Command::Ptr cmd)
{
    assert(!locked);
    nested.push_back(cmd);
}

void Meta::execute()
{
    locked = true;

    BOOST_FOREACH(Command::Ptr cmd, nested) {
        cmd->execute();
    }
}

bool Meta::undo()
{
    BOOST_FOREACH(Command::Ptr cmd, nested) {
        if(!cmd->undo()) {
//            undo_later.push_back(cmd);
        }
    }

    return true;
}

void Meta::redo()
{
    BOOST_FOREACH(Command::Ptr cmd, nested) {
        cmd->redo();
    }
}
