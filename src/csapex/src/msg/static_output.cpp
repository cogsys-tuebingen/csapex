/// HEADER
#include <csapex/msg/static_output.h>

/// COMPONENT
#include <csapex/msg/message.h>
#include <csapex/msg/input.h>
#include <csapex/model/connection.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/output_transition.h>
#include <csapex/msg/no_message.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

StaticOutput::StaticOutput(const UUID &uuid)
    : Output(uuid)
{

}

void StaticOutput::addMessage(TokenPtr message)
{
    const auto& data = message->getTokenData();
    if(!std::dynamic_pointer_cast<connection_types::MarkerMessage const>(data)) {
        setType(data->toType());
    }

    // update buffer

    apex_assert_hard(message != nullptr);
    message_to_send_ = message;
}

bool StaticOutput::hasMessage()
{
    return (bool) message_to_send_;
}


bool StaticOutput::hasMarkerMessage()
{
    if(!message_to_send_) {
        return false;
    }
    if(auto m = std::dynamic_pointer_cast<connection_types::MarkerMessage const>(message_to_send_->getTokenData())) {
        if(!std::dynamic_pointer_cast<connection_types::NoMessage const>(m)) {
            return true;
        }
    }

    return false;
}


void StaticOutput::nextMessage()
{
    setState(State::IDLE);
}

TokenPtr StaticOutput::getToken() const
{
    if(!committed_message_) {
        return Token::makeEmpty<connection_types::NoMessage>();
    } else {
        return committed_message_;
    }
}

bool StaticOutput::commitMessages(bool is_activated)
{
    activate();

    bool send_active = is_activated;


    if(message_to_send_) {
        send_active |= message_to_send_->isActive();

        committed_message_ = message_to_send_;
        clearBuffer();

    } else {
        if(!connections_.empty()) {
//            std::cout << getUUID() << " sends empty message" << std::endl;
        }
        committed_message_ = Token::makeEmpty<connection_types::NoMessage>();
    }

    ++seq_no_;

    committed_message_->setSequenceNumber(seq_no_);

    bool sent_active = false;

    if(hasActiveConnection() && send_active && !std::dynamic_pointer_cast<connection_types::NoMessage const>(committed_message_->getTokenData())) {
        std::cerr << "set an active token: " << getUUID() << std::endl;
        sent_active = true;
        committed_message_->setActive(true);
    } else {
        committed_message_->setActive(false);
    }

    ++count_;
    messageSent(this);

    return sent_active;
}

void StaticOutput::reset()
{
    Output::reset();
    committed_message_.reset();
    message_to_send_.reset();
}

void StaticOutput::disable()
{
    Output::disable();

    message_to_send_.reset();
    committed_message_.reset();
}

TokenPtr StaticOutput::getToken()
{
    return committed_message_;
}
TokenPtr StaticOutput::getAddedToken()
{
    return message_to_send_;
}

void StaticOutput::clearBuffer()
{
    message_to_send_.reset();
}
