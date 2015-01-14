/// HEADER
#include <csapex/command/dispatcher.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/graph_worker.h>
#include <csapex/view/widget_controller.h>
#include <csapex/utility/assert.h>

using namespace csapex;

CommandDispatcher::CommandDispatcher(Settings& settings, GraphWorker::Ptr graph, ThreadPool* thread_pool, NodeFactory* node_factory)
    : settings_(settings), graph_worker_(graph), thread_pool_(thread_pool), node_factory_(node_factory), dirty_(false)
{
    QObject::connect(this, SIGNAL(stateChanged()), graph_worker_->getGraph(), SIGNAL(stateChanged()));
    QObject::connect(this, SIGNAL(dirtyChanged(bool)), graph_worker_->getGraph(), SIGNAL(dirtyChanged(bool)));
}

void CommandDispatcher::reset()
{
    graph_worker_->reset();
    later.clear();
    done.clear();
    undone.clear();
    dirty_ = false;
}

void CommandDispatcher::execute(Command::Ptr command)
{
    command->init(&settings_, graph_worker_->getGraph(), thread_pool_, node_factory_);
    doExecute(command);
}

void CommandDispatcher::executeLater(Command::Ptr command)
{
    command->init(&settings_, graph_worker_->getGraph(), thread_pool_, node_factory_);
    later.push_back(command);
}

void CommandDispatcher::executeLater()
{
    Q_FOREACH(Command::Ptr cmd, later) {
        doExecute(cmd);
    }
    later.clear();
}

void CommandDispatcher::executeNotUndoable(Command::Ptr command)
{
    command->init(&settings_, graph_worker_->getGraph(), thread_pool_, node_factory_);
    Command::Access::executeCommand(graph_worker_->getGraph(), node_factory_, command);
}

void CommandDispatcher::undoNotRedoable(Command::Ptr command)
{
    command->init(&settings_, graph_worker_->getGraph(), thread_pool_, node_factory_);
    Command::Access::undoCommand(graph_worker_->getGraph(), node_factory_, command);
}


void CommandDispatcher::doExecute(Command::Ptr command)
{
    if(!command) {
        return;
    }

    if(!isDirty()) {
        command->setAfterSavepoint(true);
    }

    bool success = Command::Access::executeCommand(graph_worker_->getGraph(), node_factory_, command);
    done.push_back(command);

    while(!undone.empty()) {
        undone.pop_back();
    }

    if(success) {
        setDirty();
        Q_EMIT stateChanged();
    }
}

bool CommandDispatcher::isDirty()
{
    return dirty_;
}

void CommandDispatcher::resetDirtyPoint()
{
    setDirty(false);

    clearSavepoints();

    if(!done.empty()) {
        done.back()->setBeforeSavepoint(true);
    }
    if(!undone.empty()) {
        undone.back()->setAfterSavepoint(true);
    }

    Q_EMIT dirtyChanged(dirty_);
}

void CommandDispatcher::clearSavepoints()
{
    Q_FOREACH(Command::Ptr cmd, done) {
        cmd->setAfterSavepoint(false);
        cmd->setBeforeSavepoint(false);
    }
    Q_FOREACH(Command::Ptr cmd, undone) {
        cmd->setAfterSavepoint(false);
        cmd->setBeforeSavepoint(false);
    }
    Q_EMIT dirtyChanged(dirty_);
}

void CommandDispatcher::setDirty()
{
    setDirty(true);
}

void CommandDispatcher::setClean()
{
    setDirty(false);
}

void CommandDispatcher::setDirty(bool dirty)
{
    bool change = dirty_ != dirty;

    dirty_ = dirty;

    if(change) {
        Q_EMIT dirtyChanged(dirty_);
    }
}


bool CommandDispatcher::canUndo()
{
    return !done.empty();
}

bool CommandDispatcher::canRedo()
{
    return !undone.empty();
}

void CommandDispatcher::undo()
{
    if(!canUndo()) {
        return;
    }

    Command::Ptr last = done.back();
    done.pop_back();

    bool ret = Command::Access::undoCommand(graph_worker_->getGraph(), node_factory_, last);
    apex_assert_hard(ret);

    setDirty(!last->isAfterSavepoint());

    undone.push_back(last);

    Q_EMIT stateChanged();
}

void CommandDispatcher::redo()
{
    if(!canRedo()) {
        return;
    }

    Command::Ptr last = undone.back();
    undone.pop_back();

    Command::Access::redoCommand(graph_worker_->getGraph(), node_factory_, last);

    done.push_back(last);

    setDirty(!last->isBeforeSavepoint());

    Q_EMIT stateChanged();
}

Graph* CommandDispatcher::getGraph()
{
    return graph_worker_->getGraph();
}

void CommandDispatcher::populateDebugInfo(QTreeWidget *undo, QTreeWidget *redo)
{
    Q_FOREACH(const Command::Ptr& c, done) {
        undo->addTopLevelItem(c->createDebugInformation());
    }
    Q_FOREACH(const Command::Ptr& c, undone) {
        redo->addTopLevelItem(c->createDebugInformation());
    }
}
