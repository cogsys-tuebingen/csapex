/// HEADER
#include <csapex/msg/static_output.h>

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/msg/input.h>
#include <csapex/model/connection.h>

using namespace csapex;

StaticOutput::StaticOutput(OutputTransition* transition, const UUID &uuid)
    : Output(transition, uuid)
{

}

StaticOutput::StaticOutput(OutputTransition* transition, Unique *parent, int sub_id)
    : Output(transition, parent, sub_id)
{

}

void StaticOutput::publish(ConnectionType::ConstPtr message)
{
    setType(message->toType());

    // update buffer
    apex_assert_hard(message != nullptr);
    message_to_send_ = message;
}

bool StaticOutput::hasMessage()
{
    return (bool) message_to_send_;
}

void StaticOutput::nextMessage()
{
    setState(State::DONE);
}

ConnectionTypeConstPtr StaticOutput::getMessage() const
{
    apex_assert_hard(committed_message_ != nullptr);
    return committed_message_;
}


void StaticOutput::commitMessages()
{
    assert(canSendMessages());
    if(message_to_send_) {
        committed_message_ = message_to_send_;
        clear();

    } else {
        if(!connections_.empty()) {
            //            std::cout << getUUID() << " sends empty message" << std::endl;
        }
        committed_message_ = connection_types::makeEmpty<connection_types::NoMessage>();
    }

    committed_message_->setSequenceNumber(seq_no_);

    setState(State::ACTIVE);

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

    //    std::cerr << getUUID() << "Publish message with #" << seq_no_ << std::endl;
//    if(!targets.empty()) {
        // all connected inputs are ready to receive, send them the message
        ++count_;
//    }

    ++seq_no_;
    Q_EMIT messageSent(this);
}

void StaticOutput::reset()
{
    Output::reset();
    committed_message_.reset();
}

void StaticOutput::disable()
{
    Output::disable();

    message_to_send_.reset();
    committed_message_.reset();
}

ConnectionType::ConstPtr StaticOutput::getMessage()
{
    return committed_message_;
}

void StaticOutput::clear()
{
    message_to_send_.reset();
    setState(Output::State::RECEIVING);
}
