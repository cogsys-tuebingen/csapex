/// HEADER
#include <csapex/msg/dynamic_input.h>

/// PROJECT
#include <csapex/utility/assert.h>
#include <csapex/model/token.h>

using namespace csapex;

DynamicInput::DynamicInput(const UUID &uuid)
    : Input(uuid),
      has_last_(false)
{
    setDynamic(true);
}


std::vector<TokenPtr> DynamicInput::getMessageParts() const
{
    std::unique_lock<std::mutex> lock(message_mutex_);
    return composed_msg_;
}

void DynamicInput::setToken(TokenPtr msg)
{
    apex_assert_hard(msg != nullptr);
    std::unique_lock<std::mutex> lock(message_mutex_);
    msg_parts_.push_back(msg);

    has_last_ = msg->flags.data & (int) Token::Flags::Fields::LAST_PART;
    if(has_last_) {
        composeMessage();
    }
}


void DynamicInput::composeMessage()
{
    std::unique_lock<std::mutex> lock(message_mutex_);
    composed_msg_ = msg_parts_;
    count_++;

    msg_parts_.clear();
    has_last_ = false;

    message_set(this);
}


TokenPtr DynamicInput::getToken() const
{
    throw std::runtime_error("cannot call get message on a dynamic input");
}

bool DynamicInput::hasReceived() const
{
    std::unique_lock<std::mutex> lock(message_mutex_);
    return isConnected() && !composed_msg_.empty();
}

bool DynamicInput::hasMessage() const
{
    if(!hasReceived()) {
        return false;
    }

    std::unique_lock<std::mutex> lock(message_mutex_);
    return !composed_msg_.empty();
}

void DynamicInput::notifyMessageProcessed()
{
    composed_msg_.clear();
    Connectable::notifyMessageProcessed();
}
