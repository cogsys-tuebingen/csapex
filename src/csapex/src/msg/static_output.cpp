/// HEADER
#include <csapex/msg/static_output.h>

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/msg/input.h>

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

    setBlocked(true);
}

bool StaticOutput::hasMessage()
{
    return (bool) message_to_send_;
}

bool StaticOutput::sendMessages()
{
    assert(canSendMessages());
    if(message_to_send_) {
        message_ = message_to_send_;
        clear();

    } else {
        if(!targets_.empty()) {
//            std::cout << getUUID() << " sends empty message" << std::endl;
        }
        message_ = connection_types::makeEmpty<connection_types::NoMessage>();
    }

    message_->setSequenceNumber(seq_no_);

    // wait for all connected inputs to be able to receive
    //  * inputs can only be connected to this output since they are 1:1
    std::vector<Input*> targets;
    foreach(Input* i, targets_) {
        if(i->isEnabled()) {
            targets.push_back(i);
            assert(!i->isBlocked());
        }
    }

//    std::cerr << getUUID() << "Publish message with #" << seq_no_ << std::endl;
    if(!targets.empty()) {
        // all connected inputs are ready to receive, send them the message
        for(auto i : targets) {
            i->inputMessage(message_);
        }
        ++count_;
    }

    setBlocked(false);

    ++seq_no_;
    Q_EMIT messageSent(this);

    return false;
}

void StaticOutput::reset()
{
    Output::reset();
    message_.reset();
}

void StaticOutput::disable()
{
    Output::disable();

    //    if(isBlocked()) {
            message_to_send_.reset();
            message_.reset();
    //    }

}

ConnectionType::ConstPtr StaticOutput::getMessage()
{
    return message_;
}

void StaticOutput::clear()
{
    message_to_send_.reset();
}
