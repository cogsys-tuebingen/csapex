#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

/// COMPONENT
#include <csapex/command/command.h>

/// PROJECT
#include <utils_plugin/singleton.hpp>

/// SYSTEM
#include <deque>
#include <QObject>

namespace csapex
{

class CommandDispatcher : public QObject, public Singleton<CommandDispatcher>
{
    Q_OBJECT

    friend class Singleton<CommandDispatcher>;

public:
    static void execute(Command::Ptr command);
    static void executeLater(Command::Ptr command);
    static void executeLater();

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

    void reset();

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
    std::vector<Command::Ptr> later;

    std::deque<Command::Ptr> done;
    std::deque<Command::Ptr> undone;
    bool dirty_;
};

}

#endif // COMMAND_DISPATCHER_H
