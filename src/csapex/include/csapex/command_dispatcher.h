#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

/// COMPONENT
#include <csapex/command.h>

/// SYSTEM
#include <deque>
#include <QObject>

namespace csapex
{

class CommandDispatcher : public QObject
{
    Q_OBJECT

public:
    static CommandDispatcher& instance() {
        static CommandDispatcher inst;
        return inst;
    }

    static void execute(Command::Ptr command);

    void setGraph(Graph* graph);

    bool isDirty();

    bool canUndo();
    bool canRedo();

public Q_SLOTS:
    void undo();
    void redo();
    void setDirty();
    void setClean();
    void resetDirtyPoint();
    void clearSavepoints();

Q_SIGNALS:
    void stateChanged();
    void dirtyChanged(bool);

private:
    void doExecute(Command::Ptr command);
    void setDirty(bool dirty);

protected:
    CommandDispatcher();
    CommandDispatcher(const CommandDispatcher& copy);
    CommandDispatcher& operator = (const CommandDispatcher& assign);

private:
    std::deque<Command::Ptr> done;
    std::deque<Command::Ptr> undone;
    bool dirty_;

    Graph* graph_;
};

}

#endif // COMMAND_DISPATCHER_H
