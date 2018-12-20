/// HEADER
#include <csapex/command/dispatcher_proxy.h>

/// COMPONENT
#include <csapex/utility/assert.h>
#include <csapex/io/session.h>
#include <csapex/io/protcol/command_requests.h>
#include <csapex/io/protcol/command_broadcasts.h>

using namespace csapex;

CommandDispatcherProxy::CommandDispatcherProxy(const SessionPtr& session) : Proxy(session)
{
    dirty_ = false;
    init_ = false;
}

bool CommandDispatcherProxy::execute(const CommandPtr& command)
{
    auto result = session_->sendRequest<CommandRequests>(CommandRequests::CommandRequestType::Execute, command);
    apex_assert_hard(result);
    return result->template getResult<bool>();
}

void CommandDispatcherProxy::executeLater(const CommandPtr& command)
{
    session_->sendRequest<CommandRequests>(CommandRequests::CommandRequestType::ExecuteLater, command);
}

void CommandDispatcherProxy::executeLater()
{
    session_->sendRequest<CommandRequests>(CommandRequests::CommandRequestType::ExecuteLater);
}

bool CommandDispatcherProxy::isDirty() const
{
    if (!init_) {
        dirty_ = request<bool, CommandRequests>(CommandRequests::CommandRequestType::IsDirty);
        init_ = true;
    }

    return dirty_;
}

bool CommandDispatcherProxy::canUndo() const
{
    return request<bool, CommandRequests>(CommandRequests::CommandRequestType::CanUndo);
}

bool CommandDispatcherProxy::canRedo() const
{
    return request<bool, CommandRequests>(CommandRequests::CommandRequestType::CanRedo);
}

void CommandDispatcherProxy::undo()
{
    if (!canUndo()) {
        return;
    }
    session_->sendRequest<CommandRequests>(CommandRequests::CommandRequestType::Undo);
}

void CommandDispatcherProxy::redo()
{
    if (!canRedo()) {
        return;
    }
    session_->sendRequest<CommandRequests>(CommandRequests::CommandRequestType::Redo);
}

void CommandDispatcherProxy::handleBroadcast(const BroadcastMessageConstPtr& message)
{
    if (auto command_msg = std::dynamic_pointer_cast<CommandBroadcasts const>(message)) {
        switch (command_msg->getBroadcastType()) {
            case CommandBroadcasts::CommandBroadcastType::StateChanged:
                state_changed();
                break;

            case CommandBroadcasts::CommandBroadcastType::DirtyChanged:
                dirty_ = command_msg->getFlag();
                dirty_changed(dirty_);
                break;
            default:
                break;
        }
    }
}
