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

bool Meta::execute()
{
    locked = true;

    bool success = true;
    BOOST_FOREACH(Command::Ptr cmd, nested) {
        bool s = cmd->execute();
        if(!s) {
            std::cerr << "command failed to execute! (" << typeid(*cmd).name() << ")" << std::endl;
        }
        success &= s;
    }
    return success;
}

bool Meta::undo()
{
    BOOST_REVERSE_FOREACH(Command::Ptr cmd, nested) {
        if(!cmd->undo()) {
            undo_later.push_back(cmd);
        }
    }

    return true;
}

bool Meta::redo()
{
    bool success = true;
    BOOST_FOREACH(Command::Ptr cmd, nested) {
        success &= cmd->redo();
    }
    return success;
}
