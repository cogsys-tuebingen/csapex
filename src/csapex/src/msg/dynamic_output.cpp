/// HEADER
#include <csapex/msg/dynamic_output.h>

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/msg/input.h>
#include <csapex/model/connection.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

DynamicOutput::DynamicOutput(const UUID &uuid)
    : Output(uuid)
{
    setDynamic(true);
}

DynamicOutput::DynamicOutput(Unique *parent, int sub_id)
    : Output(parent, sub_id)
{
    setDynamic(true);
}

void DynamicOutput::publish(ConnectionType::ConstPtr message)
{
    setType(message->toType());
    messages_to_send_.push_back(message);
}

void DynamicOutput::sendMessages()
{
    apex_assert_hard(canSendMessages());

    // wait for all connected inputs to be able to receive
    //  * inputs can only be connected to this output since they are 1:1
    std::vector<Connection*> targets;
    for(ConnectionWeakPtr connection : connections_) {
        ConnectionPtr c = connection.lock();
        if(!c) {
            continue;
        }

        Input* i = dynamic_cast<Input*>(c->to());
        if(i && i->isEnabled()) {
            targets.push_back(c.get());
        }
    }

    if(!targets.empty()) {
        // all connected inputs are ready to receive, send them the message
        for(auto connection : targets) {
            for(auto msg : messages_to_send_) {
                msg->setSequenceNumber(seq_no_);
                connection->addMessage(msg);
            }
            connection->commitMessages();
        }
        ++count_;
    }

    messages_to_send_.clear();

    ++seq_no_;
    Q_EMIT messageSent(this);
}

bool DynamicOutput::hasMessage()
{
    return !messages_to_send_.empty();
}


void DynamicOutput::clear()
{
    messages_to_send_.clear();
}
