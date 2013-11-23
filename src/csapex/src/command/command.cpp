/// HEADER
#include <csapex/command/command.h>

/// COMPONENT
#include <csapex/model/graph.h>

using namespace csapex;

std::vector<Command::Ptr> Command::undo_later;

Command::Command()
    : before_save_point_(false), after_save_point_(false)
{
}

bool Command::Access::executeCommand(GraphPtr graph, Command::Ptr cmd)
{
    return Command::executeCommand(graph, cmd);
}

bool Command::Access::undoCommand(GraphPtr graph, Command::Ptr cmd)
{
    return Command::undoCommand(graph, cmd);
}

bool Command::Access::redoCommand(GraphPtr graph, Command::Ptr cmd)
{
    return Command::redoCommand(graph, cmd);
}

void Command::setGraph(Graph::Ptr graph)
{
    graph_ = graph;
}

Graph::Ptr Command::getGraph()
{
    return graph_;
}

bool Command::executeCommand(GraphPtr graph, Command::Ptr cmd)
{
    cmd->graph_ = graph;
    assert(cmd->graph_);
    return cmd->doExecute();
}

bool Command::undoCommand(GraphPtr graph, Command::Ptr cmd)
{
    cmd->graph_ = graph;
    assert(cmd->graph_);
    if(!cmd->doUndo()) {
        undo_later.push_back(cmd);
        return false;
    }

    return true;
}

bool Command::redoCommand(GraphPtr graph, Command::Ptr cmd)
{
    if(graph) {
        cmd->graph_ = graph;
    }

    assert(cmd->graph_);
    return cmd->doRedo();
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

QTreeWidgetItem* Command::createDebugInformation() const
{
    QTreeWidgetItem* tl = new QTreeWidgetItem;
    tl->setText(0, getType().c_str());
    tl->setText(1, getDescription().c_str());
    return tl;
}
