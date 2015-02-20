/// HEADER
#include <csapex/msg/dynamic_output.h>

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/msg/input.h>
#include <csapex/model/connection.h>

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

void DynamicOutput::publish(ConnectionType::ConstPtr message)
{
    setType(message->toType());
    messages_to_send_.push_back(message);
}

void DynamicOutput::commitMessages()
{
    apex_assert_hard(canSendMessages());

//    // wait for all connected inputs to be able to receive
//    //  * inputs can only be connected to this output since they are 1:1
//    std::vector<Connection*> targets;
//    for(ConnectionWeakPtr connection : connections_) {
//        ConnectionPtr c = connection.lock();
//        if(!c) {
//            continue;
//        }

//        Input* i = dynamic_cast<Input*>(c->to());
//        if(i && i->isEnabled()) {
//            targets.push_back(c.get());
//        }
//    }

//    if(!targets.empty()) {
        // all connected inputs are ready to receive, send them the message
        committed_messages_ = messages_to_send_;
        ++count_;
//    }


    messages_to_send_.clear();

    if(!committed_messages_.empty()) {
        current_message_ = committed_messages_.front();
        committed_messages_.pop_front();
    }
    setState(State::ACTIVE);

    ++seq_no_;
    Q_EMIT messageSent(this);
}

bool DynamicOutput::hasMessage()
{
    return !messages_to_send_.empty();
}

void DynamicOutput::nextMessage()
{
    if(committed_messages_.empty()) {
        setState(State::DONE);
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
