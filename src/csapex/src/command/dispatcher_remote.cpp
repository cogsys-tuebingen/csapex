/// HEADER
#include <csapex/command/dispatcher_remote.h>

/// COMPONENT
#include <csapex/utility/assert.h>
#include <csapex/io/session.h>
#include <csapex/io/protcol/command_requests.h>
#include <csapex/io/protcol/command_broadcasts.h>

using namespace csapex;

CommandDispatcherRemote::CommandDispatcherRemote(SessionPtr session)
    : session_(session)
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
        auto res = session_->sendRequest<CommandRequests>(CommandRequests::CommandRequestType::IsDirty);
        apex_assert_hard(res);
        dirty_ = res->getResult();
        init_ = true;
    }

    return dirty_;
}

bool CommandDispatcherRemote::canUndo() const
{
    auto res = session_->sendRequest<CommandRequests>(CommandRequests::CommandRequestType::CanUndo);
    apex_assert_hard(res);
    return res->getResult();
}

bool CommandDispatcherRemote::canRedo() const
{
    auto res = session_->sendRequest<CommandRequests>(CommandRequests::CommandRequestType::CanRedo);
    apex_assert_hard(res);
    return res->getResult();
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
        if(command_msg->getBroadcastType() == CommandBroadcasts::CommandBroadcastType::StateChanged) {
            state_changed();

        } else if(command_msg->getBroadcastType() == CommandBroadcasts::CommandBroadcastType::DirtyChanged) {
            dirty_ = command_msg->getFlag();
            dirty_changed(dirty_);
        }
    }
}
