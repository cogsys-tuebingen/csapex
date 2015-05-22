/// HEADER
#include <csapex/msg/dynamic_output.h>

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/msg/input.h>
#include <csapex/model/connection.h>
#include <csapex/msg/dynamic_input.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

DynamicOutput::DynamicOutput(OutputTransition* transition, const UUID &uuid)
    : Output(transition, uuid)
{
    setDynamic(true);
}

DynamicOutput::DynamicOutput(OutputTransition* transition, Unique *parent, int sub_id)
    : Output(transition, parent, sub_id)
{
    setDynamic(true);
}

void DynamicOutput::clearCorrespondents()
{
    correspondents_.clear();
}

void DynamicOutput::addCorrespondent(DynamicInput *input)
{
    correspondents_.push_back(input);
}

void DynamicOutput::publish(ConnectionType::ConstPtr message)
{
    setType(message->toType());
    messages_to_send_.push_back(message);
}

void DynamicOutput::setMultipart(bool multipart, bool last_part)
{
    // ignored since this is already dynamic
}

void DynamicOutput::commitMessages()
{
    apex_assert_hard(canSendMessages());

    setState(State::ACTIVE);

    if(messages_to_send_.empty()) {
        messages_to_send_.push_back(connection_types::makeEmpty<connection_types::NoMessage>());
    }

    committed_messages_ = messages_to_send_;
    ++count_;

    for(ConnectionTypeConstPtr& m : committed_messages_) {
        m->flags.data |= (int) ConnectionType::Flags::Fields::MULTI_PART;
    }

    apex_assert_hard(committed_messages_.size() > 0);
    committed_messages_.back()->flags.data |= (int) ConnectionType::Flags::Fields::LAST_PART;

    messages_to_send_.clear();

    ++seq_no_;


    for(DynamicInput* di : correspondents_) {
//        di->setCurrentMessageLength(committed_messages_.size(), seq_no_);
    }
    current_message_ = committed_messages_.front();
    committed_messages_.pop_front();

    Q_EMIT messageSent(this);
}

bool DynamicOutput::hasMessage()
{
    return !messages_to_send_.empty();
}

void DynamicOutput::nextMessage()
{
    if(committed_messages_.empty()) {
        setState(State::IDLE);
    } else {
        current_message_ = committed_messages_.front();
        committed_messages_.pop_front();
    }
}

ConnectionTypeConstPtr DynamicOutput::getMessage() const
{
    apex_assert_hard(current_message_ != nullptr);
    return current_message_;
}

void DynamicOutput::clear()
{
    messages_to_send_.clear();
    setState(Output::State::RECEIVING);
}
