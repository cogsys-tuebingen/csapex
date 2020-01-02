#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/command/command_fwd.h>
#include <csapex/command/command_executor.h>

/// SYSTEM
#include <deque>
#include <csapex/utility/slim_signal.h>

namespace csapex
{
class CSAPEX_COMMAND_EXPORT CommandDispatcher : public CommandExecutor
{
public:
    typedef std::shared_ptr<CommandDispatcher> Ptr;

public:
    CommandDispatcher(CsApexCore& core);

    bool execute(const CommandPtr& command) override;
    void executeLater(const CommandPtr& command) override;
    void executeLater() override;

    bool isDirty() const override;

    bool canUndo() const override;
    bool canRedo() const override;

    void undo() override;
    void redo() override;

    CommandConstPtr getNextUndoCommand() const;
    CommandConstPtr getNextRedoCommand() const;

    void visitUndoCommands(std::function<void(int level, const Command& cmd)> callback) const;
    void visitRedoCommands(std::function<void(int level, const Command& cmd)> callback) const;

    void reset();
    void setDirty();
    void setClean();
    void resetDirtyPoint();
    void clearSavepoints();

private:
    bool doExecute(Command::Ptr command);
    void setDirty(bool dirty);

protected:
    CommandDispatcher(const CommandDispatcher& copy);
    CommandDispatcher& operator=(const CommandDispatcher& assign);

private:
    CsApexCore& core_;

    std::vector<Command::Ptr> later;

    std::deque<Command::Ptr> done;
    std::deque<Command::Ptr> undone;
    bool dirty_;
};

}  // namespace csapex

#endif  // COMMAND_DISPATCHER_H
