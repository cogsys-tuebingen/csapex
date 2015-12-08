/// HEADER
#include <csapex/utility/buffer.h>

/// SYSTEM
#include <mutex>

using namespace csapex;

Buffer::Buffer()
    : enabled_(true)
{

}

Buffer::~Buffer()
{
}

ConnectionType::ConstPtr Buffer::read() const
{
    std::unique_lock<std::mutex> lock(mutex_);

    apex_assert_hard(message_);

    return message_;
}

void Buffer::write(ConnectionType::ConstPtr message)
{
    if(!enabled_) {
        return;
    }

    std::unique_lock<std::mutex> lock(mutex_);

    apex_assert_hard(!message_);
    message_ = message;
    if(!message_) {
        throw std::runtime_error("message is empty");
    }
}

void Buffer::disable()
{
    enabled_ = false;
}

void Buffer::free()
{
    std::unique_lock<std::mutex> lock(mutex_);
    message_.reset();
}

bool Buffer::isFilled() const
{
    std::unique_lock<std::mutex> lock(mutex_);
    return message_ != nullptr;
}

bool Buffer::containsNoMessage() const
{
    auto test = std::dynamic_pointer_cast<connection_types::NoMessage const>(message_);
    return test != nullptr;
}
