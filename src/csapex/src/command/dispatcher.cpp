/// HEADER
#include <csapex/command/dispatcher.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/utility/assert.h>
#include <csapex/command/command_factory.h>
#include <csapex/core/csapex_core.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

CommandDispatcher::CommandDispatcher(CsApexCore& core)
    : core_(core),
      designer_(nullptr),
      dirty_(false)
{
    stateChanged.connect([this](){ core_.getRoot()->getGraph()->state_changed(); });
}

void CommandDispatcher::setDesigner(Designer *designer)
{
    designer_ = designer;
}

void CommandDispatcher::reset()
{
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
    command->init(core_.getRoot().get(), core_, designer_);
    doExecute(command);
}

void CommandDispatcher::executeLater(Command::Ptr command)
{
    if(!command) {
        std::cerr << "trying to execute null command" << std::endl;
        return;
    }
    command->init(core_.getRoot().get(), core_, designer_);
    later.push_back(command);
}

void CommandDispatcher::executeLater()
{
    for(Command::Ptr cmd : later) {
        doExecute(cmd);
    }
    later.clear();
}

void CommandDispatcher::doExecute(Command::Ptr command)
{
    if(!command) {
        return;
    }

    if(!isDirty()) {
        command->setAfterSavepoint(true);
    }

    bool success = Command::Access::executeCommand(command);

    if(command->isUndoable()) {
        done.push_back(command);

        while(!undone.empty()) {
            undone.pop_back();
        }
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

    bool ret = Command::Access::undoCommand(last);
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

    Command::Access::redoCommand(last);

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

std::shared_ptr<command::PlaybackCommand> CommandDispatcher::make_playback(const AUUID& graph_uuid, const std::string& type) const
{
     std::shared_ptr<command::PlaybackCommand> res = std::make_shared<command::PlaybackCommand>(graph_uuid, type);
     res->init(core_.getRoot().get(), core_, designer_);
     return res;
}
