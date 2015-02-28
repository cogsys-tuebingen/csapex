/// HEADER
#include <csapex/msg/dynamic_input.h>

using namespace csapex;

DynamicInput::DynamicInput(InputTransition *transition, const UUID &uuid)
    : Input(transition, uuid), correspondent_(nullptr), current_message_length_(0)
{
    setDynamic(true);
}

DynamicInput::DynamicInput(InputTransition *transition, Unique *parent, int sub_id)
    : Input(transition, parent, sub_id), correspondent_(nullptr), current_message_length_(0)
{
    setDynamic(true);
}

void DynamicInput::setCorrespondent(DynamicOutput *output)
{
    correspondent_ = output;
}

void DynamicInput::setCurrentMessageLength(std::size_t size, int sequence_number)
{
    std::unique_lock<std::mutex> lock(message_mutex_);
    current_message_length_ = size;
    msg_parts_.clear();
    setSequenceNumber(sequence_number);
}

std::vector<ConnectionTypeConstPtr> DynamicInput::getMessageParts() const
{
    std::unique_lock<std::mutex> lock(message_mutex_);
    return composed_msg_;
}

bool DynamicInput::inputMessagePart(const ConnectionTypeConstPtr &msg)
{
    apex_assert_hard(msg != nullptr);

    std::unique_lock<std::mutex> lock(message_mutex_);
    msg_parts_.push_back(msg);

    return msg_parts_.size() == current_message_length_;
}

void DynamicInput::composeMessage()
{
    std::unique_lock<std::mutex> lock(message_mutex_);
    apex_assert_hard(msg_parts_.size() == current_message_length_);
    composed_msg_ = msg_parts_;
    count_++;

    Q_EMIT messageArrived(this);
}


ConnectionTypeConstPtr DynamicInput::getMessage() const
{
    throw std::runtime_error("cannot call get message on a dynamic input");
}

bool DynamicInput::hasReceived() const
{
    std::unique_lock<std::mutex> lock(message_mutex_);
    return isConnected() && composed_msg_.size() == current_message_length_;
}

bool DynamicInput::hasMessage() const
{
    if(!hasReceived()) {
        return false;
    }

    std::unique_lock<std::mutex> lock(message_mutex_);
    return !msg_parts_.empty();
}
