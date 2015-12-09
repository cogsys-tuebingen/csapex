/// HEADER
#include <csapex/command/command.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/utility/assert.h>

using namespace csapex;

std::vector<Command::Ptr> Command::undo_later;

Command::Command()
    : settings_(nullptr), graph_worker_(nullptr), thread_pool_(nullptr), node_factory_(nullptr),
      before_save_point_(false), after_save_point_(false)
{
}

bool Command::Access::executeCommand(GraphWorker* graph_worker, Graph* graph, ThreadPool* thread_pool, NodeFactory* node_factory, Command::Ptr cmd)
{
    return Command::executeCommand(graph_worker, graph, thread_pool, node_factory, cmd);
}

bool Command::Access::undoCommand(GraphWorker* graph_worker, Graph* graph, ThreadPool* thread_pool, NodeFactory* node_factory, Command::Ptr cmd)
{
    return Command::undoCommand(graph_worker, graph, thread_pool, node_factory, cmd);
}

bool Command::Access::redoCommand(GraphWorker* graph_worker, Graph* graph, ThreadPool* thread_pool, NodeFactory* node_factory, Command::Ptr cmd)
{
    return Command::redoCommand(graph_worker, graph, thread_pool, node_factory, cmd);
}

void Command::init(Settings *settings, GraphWorker* graph_worker, Graph* graph, ThreadPool *thread_pool, NodeFactory* node_factory)
{
    apex_assert_hard(settings);
    apex_assert_hard(graph_worker);
    apex_assert_hard(thread_pool);
    apex_assert_hard(node_factory);

    settings_ = settings;
    graph_worker_ = graph_worker;
    graph_ = graph;
    thread_pool_ = thread_pool;
    node_factory_ = node_factory;
}

bool Command::executeCommand(GraphWorker* graph_worker, Graph* graph, ThreadPool* thread_pool, NodeFactory* node_factory, Command::Ptr cmd)
{
    apex_assert_hard(graph_worker);
    apex_assert_hard(thread_pool);
    apex_assert_hard(node_factory);

    cmd->graph_worker_ = graph_worker;
    cmd->graph_ = graph;
    cmd->thread_pool_ = thread_pool;
    cmd->node_factory_ = node_factory;
    return cmd->doExecute();
}

bool Command::undoCommand(GraphWorker* graph_worker, Graph* graph, ThreadPool* thread_pool, NodeFactory* node_factory, Command::Ptr cmd)
{
    apex_assert_hard(graph_worker);
    apex_assert_hard(thread_pool);
    apex_assert_hard(node_factory);

    cmd->graph_worker_ = graph_worker;
    cmd->graph_ = graph;
    cmd->thread_pool_ = thread_pool;
    cmd->node_factory_ = node_factory;
    if(!cmd->doUndo()) {
        undo_later.push_back(cmd);
        return false;
    }

    return true;
}

bool Command::redoCommand(GraphWorker* graph_worker, Graph* graph, ThreadPool* thread_pool, NodeFactory* node_factory, Command::Ptr cmd)
{
    apex_assert_hard(graph_worker);
    apex_assert_hard(thread_pool);
    apex_assert_hard(node_factory);

    cmd->graph_worker_ = graph_worker;
    cmd->graph_ = graph;
    cmd->thread_pool_ = thread_pool;
    cmd->node_factory_ = node_factory;

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
