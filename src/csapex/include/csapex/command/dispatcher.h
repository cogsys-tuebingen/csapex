#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/command/command_fwd.h>

/// SYSTEM
#include <deque>
#include <csapex/utility/slim_signal.h>

namespace csapex
{

class CommandDispatcher
{
public:
    typedef std::shared_ptr<CommandDispatcher> Ptr;

public:
    CommandDispatcher(Settings& settings, GraphFacadePtr graph_facade,
                      GraphPtr graph,
                      ThreadPool* thread_pool, NodeFactory* node_factory);

    void execute(Command::Ptr command);
    void executeLater(Command::Ptr command);
    void executeLater();

    bool isDirty() const;

    bool canUndo() const;
    bool canRedo() const;

    void undo();
    void redo();

    CommandConstPtr getNextUndoCommand() const;
    CommandConstPtr getNextRedoCommand() const;

    Graph* getGraph();
    CommandFactory* getCommandFactory();

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
    csapex::slim_signal::Signal<void()> stateChanged;
    csapex::slim_signal::Signal<void(bool)> dirtyChanged;

private:
    void doExecute(Command::Ptr command);
    void setDirty(bool dirty);

protected:
    CommandDispatcher(const CommandDispatcher& copy);
    CommandDispatcher& operator = (const CommandDispatcher& assign);

private:
    Settings& settings_;
    GraphFacadePtr graph_facade_;
    GraphPtr graph_;
    ThreadPool* thread_pool_;
    NodeFactory* node_factory_;

    CommandFactoryPtr cmd_factory_;

    std::vector<Command::Ptr> later;

    std::deque<Command::Ptr> done;
    std::deque<Command::Ptr> undone;
    bool dirty_;
};

}

#endif // COMMAND_DISPATCHER_H
