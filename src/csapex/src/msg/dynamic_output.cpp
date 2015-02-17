/// HEADER
#include <csapex/msg/dynamic_output.h>

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/msg/input.h>
#include <csapex/model/connection.h>

using namespace csapex;

DynamicOutput::DynamicOutput(const UUID &uuid)
    : Output(uuid)
{

}

DynamicOutput::DynamicOutput(Unique *parent, int sub_id)
    : Output(parent, sub_id)
{

}

void DynamicOutput::publish(ConnectionType::ConstPtr message)
{
    setType(message->toType());
    messages_to_send_.push_back(message);
    setBlocked(true);
}

bool DynamicOutput::sendMessages()
{
    assert(canSendMessages());

    ConnectionType::ConstPtr msg;
    if(messages_to_send_.empty()) {
        msg = connection_types::makeEmpty<connection_types::NoMessage>();
    } else {
        msg = messages_to_send_.front();
        messages_to_send_.pop_front();
    }

    msg->setSequenceNumber(seq_no_);

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
            assert(!i->isBlocked());
        }
    }

//    std::cerr << getUUID() << "Publish message with #" << seq_no_ << std::endl;
    if(!targets.empty()) {
        // all connected inputs are ready to receive, send them the message
        for(auto i : targets) {
            i->inputMessage(msg);
        }
        ++count_;
    }


    if(messages_to_send_.empty() || targets.empty()) {
        setBlocked(false);
        messages_to_send_.clear();

        ++seq_no_;
        Q_EMIT messageSent(this);
    }

    return !messages_to_send_.empty();
}

bool DynamicOutput::hasMessage()
{
    return !messages_to_send_.empty();
}


void DynamicOutput::clear()
{
    messages_to_send_.clear();
}
