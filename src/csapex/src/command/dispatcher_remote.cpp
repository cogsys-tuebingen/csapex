/// HEADER
#include <csapex/command/dispatcher_remote.h>

/// COMPONENT
#include <csapex/utility/assert.h>
#include <csapex/io/session.h>
#include <csapex/io/protcol/command_requests.h>
#include <csapex/io/protcol/command_broadcasts.h>

using namespace csapex;

CommandDispatcherRemote::CommandDispatcherRemote(const SessionPtr& session)
    : Remote(session)
{
    dirty_ = false;
    init_ = false;
}

void CommandDispatcherRemote::execute(const CommandPtr &command)
{
    session_->sendRequest<CommandRequests>(CommandRequests::CommandRequestType::Execute, command);
}

void CommandDispatcherRemote::executeLater(const CommandPtr &command)
{
    session_->sendRequest<CommandRequests>(CommandRequests::CommandRequestType::ExecuteLater, command);
}

void CommandDispatcherRemote::executeLater()
{
    session_->sendRequest<CommandRequests>(CommandRequests::CommandRequestType::ExecuteLater);
}

bool CommandDispatcherRemote::isDirty() const
{
    if(!init_) {
        dirty_ = request<bool, CommandRequests>(CommandRequests::CommandRequestType::IsDirty);
        init_ = true;
    }

    return dirty_;
}

bool CommandDispatcherRemote::canUndo() const
{
    return request<bool, CommandRequests>(CommandRequests::CommandRequestType::CanUndo);
}

bool CommandDispatcherRemote::canRedo() const
{
    return request<bool, CommandRequests>(CommandRequests::CommandRequestType::CanRedo);
}

void CommandDispatcherRemote::undo()
{
    if(!canUndo()) {
        return;
    }
    session_->sendRequest<CommandRequests>(CommandRequests::CommandRequestType::Undo);
}

void CommandDispatcherRemote::redo()
{
    if(!canRedo()) {
        return;
    }
    session_->sendRequest<CommandRequests>(CommandRequests::CommandRequestType::Redo);
}

void CommandDispatcherRemote::handleBroadcast(const BroadcastMessageConstPtr& message)
{
    if(auto command_msg = std::dynamic_pointer_cast<CommandBroadcasts const>(message)) {
        switch(command_msg->getBroadcastType()) {
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
