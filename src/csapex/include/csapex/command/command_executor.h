#ifndef COMMAND_EXECUTOR_H
#define COMMAND_EXECUTOR_H

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/command/command_fwd.h>

/// SYSTEM
#include <deque>
#include <csapex/utility/slim_signal.h>

namespace csapex
{

class CSAPEX_COMMAND_EXPORT CommandExecutor
{
public:
    virtual ~CommandExecutor() = default;


    virtual void execute(const CommandPtr& command) = 0;
    virtual void executeLater(const CommandPtr&  command) = 0;
    virtual void executeLater() = 0;

    virtual bool isDirty() const = 0;

    virtual bool canUndo() const = 0;
    virtual bool canRedo() const = 0;

    virtual void undo() = 0;
    virtual void redo() = 0;

public:
    slim_signal::Signal<void()> state_changed;
    slim_signal::Signal<void(bool)> dirty_changed;
};

}

#endif // COMMAND_EXECUTOR_H
