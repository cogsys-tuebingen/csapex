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

int Meta::commands() const
{
    return nested.size();
}

bool Meta::execute(Graph& graph)
{
    locked = true;

    bool change = true;
    BOOST_FOREACH(Command::Ptr cmd, nested) {
        change &= cmd->execute(graph);
    }
    return change;
}

bool Meta::undo(Graph& graph)
{
    BOOST_REVERSE_FOREACH(Command::Ptr cmd, nested) {
        if(!cmd->undo(graph)) {
            undo_later.push_back(cmd);
        }
    }

    return true;
}

bool Meta::redo(Graph& graph)
{
    bool change = true;
    BOOST_FOREACH(Command::Ptr cmd, nested) {
        change &= cmd->redo(graph);
    }
    return change;
}
