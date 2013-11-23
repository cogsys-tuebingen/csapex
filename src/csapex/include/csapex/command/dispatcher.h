#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

/// COMPONENT
#include <csapex/command/command.h>

/// PROJECT
#include <utils_plugin/singleton.hpp>

/// SYSTEM
#include <deque>
#include <QObject>
#include <QTreeWidget>

namespace csapex
{

class CommandDispatcher : public QObject
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<CommandDispatcher> Ptr;

public:
    CommandDispatcher();

    void execute(Command::Ptr command);
    void executeLater(Command::Ptr command);
    void executeLater();

    bool isDirty();

    bool canUndo();
    bool canRedo();

    void undo();
    void redo();

    GraphPtr getGraph();

    void executeNotUndoable(Command::Ptr command);

    void populateDebugInfo(QTreeWidget* undo, QTreeWidget *redo);

public Q_SLOTS:
    void setDirty();
    void setClean();
    void resetDirtyPoint();
    void clearSavepoints();

    void reset();

Q_SIGNALS:
    void stateChanged();
    void dirtyChanged(bool);

private:
    void doExecute(Command::Ptr command);
    void setDirty(bool dirty);

protected:
    CommandDispatcher(const CommandDispatcher& copy);
    CommandDispatcher& operator = (const CommandDispatcher& assign);

private:
    GraphPtr graph_;

    std::vector<Command::Ptr> later;

    std::deque<Command::Ptr> done;
    std::deque<Command::Ptr> undone;
    bool dirty_;
};

}

#endif // COMMAND_DISPATCHER_H
