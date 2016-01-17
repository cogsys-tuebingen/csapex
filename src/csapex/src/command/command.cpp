/// HEADER
#include <csapex/command/command.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/utility/assert.h>

using namespace csapex;

std::vector<Command::Ptr> Command::undo_later;

Command::Command()
    : settings_(nullptr), node_factory_(nullptr),
      graph_facade_(nullptr), thread_pool_(nullptr),
      before_save_point_(false), after_save_point_(false),
      initialized_(false)
{
}

bool Command::Access::executeCommand(Command::Ptr cmd)
{
    return Command::executeCommand(cmd);
}

bool Command::Access::undoCommand(Command::Ptr cmd)
{
    return Command::undoCommand(cmd);
}

bool Command::Access::redoCommand(Command::Ptr cmd)
{
    return Command::redoCommand(cmd);
}

void Command::init(Settings *settings, GraphFacade* graph_facade, ThreadPool *thread_pool, NodeFactory* node_factory)
{
    apex_assert_hard(settings);
    apex_assert_hard(graph_facade);
    apex_assert_hard(thread_pool);
    apex_assert_hard(node_factory);

    settings_ = settings;
    graph_facade_ = graph_facade;
    thread_pool_ = thread_pool;
    node_factory_ = node_factory;

    initialized_ = true;
}

bool Command::executeCommand(Command::Ptr cmd)
{
    apex_assert_hard(cmd->graph_facade_);
    apex_assert_hard(cmd->thread_pool_);
    apex_assert_hard(cmd->node_factory_);

    return cmd->doExecute();
}

bool Command::undoCommand(Command::Ptr cmd)
{
    apex_assert_hard(cmd->graph_facade_);
    apex_assert_hard(cmd->thread_pool_);
    apex_assert_hard(cmd->node_factory_);

    if(!cmd->doUndo()) {
        undo_later.push_back(cmd);
        return false;
    }

    return true;
}

bool Command::redoCommand(Command::Ptr cmd)
{
    apex_assert_hard(cmd->graph_facade_);
    apex_assert_hard(cmd->thread_pool_);
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

void Command::accept(int level, std::function<void (int level, const Command &)> callback) const
{
    callback(level, *this);
}

GraphFacade* Command::getGraphFacade()
{
    return graph_facade_;
}
GraphFacade* Command::getGraphFacade(const UUID& graph_id)
{
    return graph_facade_->getSubGraph(graph_id);
}
Graph* Command::getGraph()
{
    return graph_facade_->getGraph();
}
ThreadPool* Command::getThreadPool()
{
    return thread_pool_;
}
