/// HEADER
#include <csapex/command/dispatcher.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/utility/assert.h>
#include <csapex/command/command_factory.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

CommandDispatcher::CommandDispatcher(Settings& settings, GraphFacade::Ptr graph_facade,
                                     GraphPtr graph,
                                     ThreadPool* thread_pool, NodeFactory* node_factory)
    : settings_(settings), graph_facade_(graph_facade), graph_(graph), thread_pool_(thread_pool), node_factory_(node_factory),
      cmd_factory_(std::make_shared<CommandFactory>(graph.get())),
      dirty_(false)
{
    stateChanged.connect([this](){ graph_->stateChanged(); });
}

void CommandDispatcher::reset()
{
    graph_facade_->reset();
    later.clear();
    done.clear();
    undone.clear();
    dirty_ = false;
}

void CommandDispatcher::execute(Command::Ptr command)
{
    if(!command) {
        std::cerr << "trying to execute null command" << std::endl;
        return;
    }
    command->init(&settings_, graph_facade_.get(), graph_.get(), thread_pool_, node_factory_);
    doExecute(command);
}

void CommandDispatcher::executeLater(Command::Ptr command)
{
    if(!command) {
        std::cerr << "trying to execute null command" << std::endl;
        return;
    }
    command->init(&settings_, graph_facade_.get(), graph_.get(), thread_pool_, node_factory_);
    later.push_back(command);
}

void CommandDispatcher::executeLater()
{
    for(Command::Ptr cmd : later) {
        doExecute(cmd);
    }
    later.clear();
}

void CommandDispatcher::executeNotUndoable(Command::Ptr command)
{
    command->init(&settings_, graph_facade_.get(), graph_.get(), thread_pool_, node_factory_);
    Command::Access::executeCommand(graph_facade_.get(), graph_.get(), thread_pool_, node_factory_, command);
}

void CommandDispatcher::undoNotRedoable(Command::Ptr command)
{
    command->init(&settings_, graph_facade_.get(), graph_.get(), thread_pool_, node_factory_);
    Command::Access::undoCommand(graph_facade_.get(), graph_.get(), thread_pool_, node_factory_, command);
}


void CommandDispatcher::doExecute(Command::Ptr command)
{
    if(!command) {
        return;
    }

    if(!isDirty()) {
        command->setAfterSavepoint(true);
    }

    bool success = Command::Access::executeCommand(graph_facade_.get(), graph_.get(), thread_pool_, node_factory_, command);
    done.push_back(command);

    while(!undone.empty()) {
        undone.pop_back();
    }

    if(success) {
        setDirty();
        stateChanged();
    }
}

bool CommandDispatcher::isDirty() const
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

    dirtyChanged(dirty_);
}

void CommandDispatcher::clearSavepoints()
{
    for(Command::Ptr cmd : done) {
        cmd->setAfterSavepoint(false);
        cmd->setBeforeSavepoint(false);
    }
    for(Command::Ptr cmd : undone) {
        cmd->setAfterSavepoint(false);
        cmd->setBeforeSavepoint(false);
    }
    dirtyChanged(dirty_);
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
        dirtyChanged(dirty_);
    }
}


bool CommandDispatcher::canUndo() const
{
    return !done.empty();
}

bool CommandDispatcher::canRedo() const
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

    bool ret = Command::Access::undoCommand(graph_facade_.get(), graph_.get(), thread_pool_, node_factory_, last);
    apex_assert_hard(ret);

    setDirty(!last->isAfterSavepoint());

    undone.push_back(last);

    stateChanged();
}

void CommandDispatcher::redo()
{
    if(!canRedo()) {
        return;
    }

    Command::Ptr last = undone.back();
    undone.pop_back();

    Command::Access::redoCommand(graph_facade_.get(), graph_.get(), thread_pool_, node_factory_, last);

    done.push_back(last);

    setDirty(!last->isBeforeSavepoint());

    stateChanged();
}

CommandConstPtr CommandDispatcher::getNextUndoCommand() const
{
    if(canUndo()) {
        return done.back();
    } else {
        return nullptr;
    }
}

CommandConstPtr CommandDispatcher::getNextRedoCommand() const
{
    if(canRedo()) {
        return undone.back();
    } else {
        return nullptr;
    }
}

Graph* CommandDispatcher::getGraph()
{
    return graph_.get();
}

CommandFactory* CommandDispatcher::getCommandFactory()
{
    return cmd_factory_.get();
}

void CommandDispatcher::visitUndoCommands(std::function<void (int level, const Command &)> callback) const
{
    for(const Command::Ptr& c : done) {
        c->accept(0, callback);
    }
}

void CommandDispatcher::visitRedoCommands(std::function<void (int level, const Command &)> callback) const
{
    for(const Command::Ptr& c : undone) {
        c->accept(0, callback);
    }
}
