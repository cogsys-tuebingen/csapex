/// HEADER
#include <csapex/command.h>

using namespace csapex;

std::vector<Command::Ptr> Command::undo_later;

Command::Command()
{
}

void Command::doExecute(Command::Ptr other)
{
    other->execute();
}
bool Command::doUndo(Command::Ptr other)
{
    if(!other->undo()) {
        undo_later.push_back(other);
        return false;
    }

    return true;
}

void Command::doRedo(Command::Ptr other)
{
    other->redo();
}
