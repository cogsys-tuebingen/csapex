/// HEADER
#include <csapex/command/command.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <QTreeWidgetItem>

using namespace csapex;

std::vector<Command::Ptr> Command::undo_later;

Command::Command()
    : settings_(nullptr), before_save_point_(false), after_save_point_(false)
{
}

bool Command::Access::executeCommand(Graph* graph, NodeFactory* node_factory, Command::Ptr cmd)
{
    return Command::executeCommand(graph, node_factory, cmd);
}

bool Command::Access::undoCommand(Graph* graph, NodeFactory* node_factory, Command::Ptr cmd)
{
    return Command::undoCommand(graph, node_factory, cmd);
}

bool Command::Access::redoCommand(Graph* graph, NodeFactory* node_factory, Command::Ptr cmd)
{
    return Command::redoCommand(graph, node_factory, cmd);
}

void Command::init(Settings *settings, Graph* graph, ThreadPool *thread_pool, NodeFactory* node_factory)
{
    settings_ = settings;
    graph_ = graph;
    thread_pool_ = thread_pool;
    node_factory_ = node_factory;
}

bool Command::executeCommand(Graph* graph, NodeFactory* node_factory, Command::Ptr cmd)
{
    apex_assert_hard(graph);
    apex_assert_hard(node_factory);
    cmd->graph_ = graph;
    cmd->node_factory_ = node_factory;
    return cmd->doExecute();
}

bool Command::undoCommand(Graph* graph, NodeFactory* node_factory, Command::Ptr cmd)
{
    apex_assert_hard(graph);
    apex_assert_hard(node_factory);
    cmd->graph_ = graph;
    cmd->node_factory_ = node_factory;
    if(!cmd->doUndo()) {
        undo_later.push_back(cmd);
        return false;
    }

    return true;
}

bool Command::redoCommand(Graph* graph, NodeFactory* node_factory, Command::Ptr cmd)
{
    cmd->graph_ = graph;
    cmd->node_factory_ = node_factory;
    apex_assert_hard(cmd->graph_);
    apex_assert_hard(cmd->node_factory_);
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
