/// HEADER
#include <csapex/msg/dynamic_input.h>

using namespace csapex;

DynamicInput::DynamicInput(InputTransition *transition, const UUID &uuid)
    : Input(transition, uuid), correspondent_(nullptr)
{
    setDynamic(true);
}

DynamicInput::DynamicInput(InputTransition *transition, Unique *parent, int sub_id)
    : Input(transition, parent, sub_id), correspondent_(nullptr)
{
    setDynamic(true);
}

void DynamicInput::setCorrespondent(DynamicOutput *output)
{
    correspondent_ = output;
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

    composed_msg_.clear();

    return msg->flags.data & (int) ConnectionType::Flags::Fields::LAST_PART;
}

void DynamicInput::composeMessage()
{
    std::unique_lock<std::mutex> lock(message_mutex_);
    composed_msg_ = msg_parts_;
    count_++;

    msg_parts_.clear();

    Q_EMIT messageArrived(this);
}


ConnectionTypeConstPtr DynamicInput::getMessage() const
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
