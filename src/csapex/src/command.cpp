/// HEADER
#include <csapex/command.h>

/// COMPONENT
#include <csapex/graph.h>

using namespace csapex;

std::vector<Command::Ptr> Command::undo_later;

Command::Command()
    : after_save_point_(false), before_save_point_(false)
{
}

bool Command::doExecute(Graph& graph, Command::Ptr other)
{
    return other->execute(graph);
}
bool Command::doUndo(Graph& graph, Command::Ptr other)
{
    if(!other->undo(graph)) {
        undo_later.push_back(other);
        return false;
    }

    return true;
}

bool Command::doRedo(Graph &graph, Command::Ptr other)
{
    return other->redo(graph);
}

void Command::setAfterSavepoint(bool save)
{
    after_save_point_ = save;
}

bool Command::isAfterSavepoint()
{
    return after_save_point_;
}

void Command::setBeforeSavepoint(bool save)
{
    before_save_point_ = save;
}

bool Command::isBeforeSavepoint()
{
    return before_save_point_;
}
