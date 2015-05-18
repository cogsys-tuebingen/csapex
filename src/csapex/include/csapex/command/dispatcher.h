#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

/// COMPONENT
#include <csapex/command/command.h>

/// SYSTEM
#include <deque>
#include <boost/signals2/signal.hpp>

namespace csapex
{

class CommandDispatcher
{
public:
    typedef std::shared_ptr<CommandDispatcher> Ptr;

public:
    CommandDispatcher(Settings& settings, GraphWorkerPtr graph, ThreadPool* thread_pool, NodeFactory* node_factory);

    void execute(Command::Ptr command);
    void executeLater(Command::Ptr command);
    void executeLater();

    bool isDirty();

    bool canUndo();
    bool canRedo();

    void undo();
    void redo();

    Graph* getGraph();

    void executeNotUndoable(Command::Ptr command);
    void undoNotRedoable(Command::Ptr command);

    void visitUndoCommands(std::function<void(int level, const Command& cmd)> callback) const;
    void visitRedoCommands(std::function<void(int level, const Command& cmd)> callback) const;

    void reset();
    void setDirty();
    void setClean();
    void resetDirtyPoint();
    void clearSavepoints();

public:
    boost::signals2::signal<void()> stateChanged;
    boost::signals2::signal<void(bool)> dirtyChanged;

private:
    void doExecute(Command::Ptr command);
    void setDirty(bool dirty);

protected:
    CommandDispatcher(const CommandDispatcher& copy);
    CommandDispatcher& operator = (const CommandDispatcher& assign);

private:
    Settings& settings_;
    GraphWorkerPtr graph_worker_;
    ThreadPool* thread_pool_;
    NodeFactory* node_factory_;

    std::vector<Command::Ptr> later;

    std::deque<Command::Ptr> done;
    std::deque<Command::Ptr> undone;
    bool dirty_;
};

}

#endif // COMMAND_DISPATCHER_H
