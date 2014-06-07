/// HEADER
#include <csapex/command/dispatcher.h>

/// COMPONENT
#include <csapex/model/graph.h>
#include <csapex/view/widget_controller.h>

using namespace csapex;

CommandDispatcher::CommandDispatcher(Graph::Ptr graph, WidgetController::Ptr widget_control)
    : graph_(graph), widget_ctrl_(widget_control), dirty_(false)
{
    graph_->init(this);
    widget_ctrl_->setCommandDispatcher(this);

    QObject::connect(this, SIGNAL(stateChanged()), graph_.get(), SIGNAL(stateChanged()));
    QObject::connect(this, SIGNAL(dirtyChanged(bool)), graph_.get(), SIGNAL(dirtyChanged(bool)));
}

void CommandDispatcher::reset()
{
    graph_->reset();
    later.clear();
    done.clear();
    undone.clear();
    dirty_ = false;
}

void CommandDispatcher::execute(Command::Ptr command)
{
    doExecute(command);
}

void CommandDispatcher::executeLater(Command::Ptr command)
{
    command->setGraph(graph_);
    command->setWidgetController(widget_ctrl_);
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
    Command::Access::executeCommand(graph_, widget_ctrl_, command);
}


void CommandDispatcher::doExecute(Command::Ptr command)
{
    if(!command) {
        return;
    }

    if(!isDirty()) {
        command->setAfterSavepoint(true);
    }

    bool success = Command::Access::executeCommand(graph_, widget_ctrl_, command);
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

    bool ret = Command::Access::undoCommand(graph_, widget_ctrl_, last);
    assert(ret);

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

    Command::Access::redoCommand(graph_, widget_ctrl_, last);

    done.push_back(last);

    setDirty(!last->isBeforeSavepoint());

    Q_EMIT stateChanged();
}

Graph::Ptr CommandDispatcher::getGraph()
{
    return graph_;
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
