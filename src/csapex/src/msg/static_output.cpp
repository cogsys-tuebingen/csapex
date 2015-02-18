/// HEADER
#include <csapex/msg/static_output.h>

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/msg/input.h>
#include <csapex/model/connection.h>

using namespace csapex;

StaticOutput::StaticOutput(const UUID &uuid)
    : Output(uuid)
{

}

StaticOutput::StaticOutput(Unique *parent, int sub_id)
    : Output(parent, sub_id)
{

}

void StaticOutput::publish(ConnectionType::ConstPtr message)
{
    setType(message->toType());

    // update buffer
    message_to_send_ = message;
}

bool StaticOutput::hasMessage()
{
    return (bool) message_to_send_;
}

void StaticOutput::sendMessages()
{
    assert(canSendMessages());
    if(message_to_send_) {
        message_ = message_to_send_;
        clear();

    } else {
        if(!connections_.empty()) {
            //            std::cout << getUUID() << " sends empty message" << std::endl;
        }
        message_ = connection_types::makeEmpty<connection_types::NoMessage>();
    }

    message_->setSequenceNumber(seq_no_);

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

    //    std::cerr << getUUID() << "Publish message with #" << seq_no_ << std::endl;
    if(!targets.empty()) {
        // all connected inputs are ready to receive, send them the message
        for(auto i : targets) {
            i->addMessage(message_);
            i->commitMessages();
        }
        ++count_;
    }

    ++seq_no_;
    Q_EMIT messageSent(this);
}

void StaticOutput::reset()
{
    Output::reset();
    message_.reset();
}

void StaticOutput::disable()
{
    Output::disable();

    message_to_send_.reset();
    message_.reset();
}

ConnectionType::ConstPtr StaticOutput::getMessage()
{
    return message_;
}

void StaticOutput::clear()
{
    message_to_send_.reset();
}
