#ifndef DISPATCHER_REMOTE_H
#define DISPATCHER_REMOTE_H

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/command/command_fwd.h>
#include <csapex/command/command_executor.h>
#include <csapex/io/io_fwd.h>

/// SYSTEM
#include <deque>
#include <csapex/utility/slim_signal.h>

namespace csapex
{

class CSAPEX_COMMAND_EXPORT CommandDispatcherRemote : public CommandExecutor
{
public:
    typedef std::shared_ptr<CommandDispatcherRemote> Ptr;

public:
    CommandDispatcherRemote(SessionPtr session);

    void execute(const CommandPtr& command);
    void executeLater(const CommandPtr&  command);
    void executeLater();

    bool isDirty() const;

    bool canUndo() const;
    bool canRedo() const;

    void undo();
    void redo();

    void handleBroadcast(const BroadcastMessageConstPtr& message);

private:
    SessionPtr session_;

    mutable bool init_;
    mutable bool dirty_;
};

}

#endif // DISPATCHER_REMOTE_H
