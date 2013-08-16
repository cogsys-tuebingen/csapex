/// HEADER
#include <csapex/command_dispatcher.h>

using namespace csapex;

CommandDispatcher::CommandDispatcher()
    : dirty_(false), graph_(NULL)
{

}

void CommandDispatcher::setGraph(Graph *graph)
{
    graph_ = graph;
}


void CommandDispatcher::execute(Command::Ptr command)
{
    instance().doExecute(command);
}

void CommandDispatcher::executeLater(Command::Ptr command)
{
    instance().later.push_back(command);
}

void CommandDispatcher::executeLater()
{
    foreach(Command::Ptr cmd, instance().later) {
        instance().doExecute(cmd);
    }
    instance().later.clear();
}

void CommandDispatcher::doExecute(Command::Ptr command)
{
    if(!isDirty()) {
        command->setAfterSavepoint(true);
    }

    bool change = command->execute();
    done.push_back(command);

    while(!undone.empty()) {
        undone.pop_back();
    }

    if(change) {
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
}

void CommandDispatcher::clearSavepoints()
{
    foreach(Command::Ptr cmd, done) {
        cmd->setAfterSavepoint(false);
        cmd->setBeforeSavepoint(false);
    }
    foreach(Command::Ptr cmd, undone) {
        cmd->setAfterSavepoint(false);
        cmd->setBeforeSavepoint(false);
    }
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

    bool ret = last->undo();
    assert(ret);

//    while(!Command::undo_later.empty()) {
//        for(std::vector<Command::Ptr>::iterator it = Command::undo_later.begin(); it != Command::undo_later.end();) {
//            if((*it)->undo(*graph_)) {
//                it = Command::undo_later.erase(it);

//            } else {
//                ++it;
//            }
//        }
//    }

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

    last->redo();

    done.push_back(last);

    setDirty(!last->isBeforeSavepoint());

    Q_EMIT stateChanged();
}

