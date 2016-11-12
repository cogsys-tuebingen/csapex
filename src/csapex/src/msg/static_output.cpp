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

StaticOutput::StaticOutput(const UUID &uuid, ConnectableOwnerWeakPtr owner)
    : Output(uuid, owner)
{

}

void StaticOutput::addMessage(TokenPtr message)
{
    const auto& data = message->getTokenData();
    if(!std::dynamic_pointer_cast<connection_types::MarkerMessage const>(data)) {
        setType(data->toType());
    }

    // update buffer

    std::unique_lock<std::recursive_mutex> lock(message_mutex_);
    apex_assert_hard(message != nullptr);
    message_to_send_ = message;
}

bool StaticOutput::hasMessage()
{
    std::unique_lock<std::recursive_mutex> lock(message_mutex_);
    return (bool) message_to_send_;
}


bool StaticOutput::hasMarkerMessage()
{
    std::unique_lock<std::recursive_mutex> lock(message_mutex_);
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
    std::unique_lock<std::recursive_mutex> lock(message_mutex_);

    if(!committed_message_) {
        return Token::makeEmpty<connection_types::NoMessage>();
    } else {
        return committed_message_;
    }
}

bool StaticOutput::commitMessages(bool is_activated)
{
    activate();

    bool send_activator = is_activated;
    bool send_deactivator = false;

    bool sent_activator_message = false;

    {
        std::unique_lock<std::recursive_mutex> lock(message_mutex_);
        if(message_to_send_) {
            apex_assert_hard(message_to_send_.get() != committed_message_.get());

            send_activator |= message_to_send_->getActivityModifier() == ActivityModifier::ACTIVATE;
            send_deactivator |= message_to_send_->getActivityModifier() == ActivityModifier::DEACTIVATE;

            //committed_message_ = message_to_send_;
            committed_message_.reset();
            committed_message_ = message_to_send_;
            message_to_send_.reset();
            clearBuffer();

        } else {
            if(!connections_.empty()) {
                //            std::cout << getUUID() << " sends empty message" << std::endl;
            }
            committed_message_ = Token::makeEmpty<connection_types::NoMessage>();
        }

        ++seq_no_;

        committed_message_->setSequenceNumber(seq_no_);
        if(hasActiveConnection() && (send_activator || send_deactivator) && !std::dynamic_pointer_cast<connection_types::NoMessage const>(committed_message_->getTokenData())) {
            std::cerr << "set an active token: " << getUUID() << std::endl;
            sent_activator_message = true;
            if(send_activator) {
                committed_message_->setActivityModifier(ActivityModifier::ACTIVATE);
            } else {
                committed_message_->setActivityModifier(ActivityModifier::DEACTIVATE);
            }
        } else {
            committed_message_->setActivityModifier(ActivityModifier::NONE);
        }

        ++count_;
    }
    messageSent(this);

    return sent_activator_message;
}

void StaticOutput::reset()
{
    Output::reset();

    std::unique_lock<std::recursive_mutex> lock(message_mutex_);
    committed_message_.reset();
    message_to_send_.reset();
}

void StaticOutput::disable()
{
    Output::disable();

    std::unique_lock<std::recursive_mutex> lock(message_mutex_);
    message_to_send_.reset();
    committed_message_.reset();
}

TokenPtr StaticOutput::getToken()
{
    std::unique_lock<std::recursive_mutex> lock(message_mutex_);
    return committed_message_;
}
TokenPtr StaticOutput::getAddedToken()
{
    std::unique_lock<std::recursive_mutex> lock(message_mutex_);
    return message_to_send_;
}

void StaticOutput::clearBuffer()
{
    std::unique_lock<std::recursive_mutex> lock(message_mutex_);
    message_to_send_.reset();
}
