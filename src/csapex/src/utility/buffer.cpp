/// HEADER
#include <csapex/utility/buffer.h>

/// SYSTEM
#include <mutex>

using namespace csapex;

Buffer::Buffer()
    : enabled_(true), mutex_(new std::mutex)
{

}

Buffer::~Buffer()
{
    delete mutex_;
}

ConnectionType::ConstPtr Buffer::readImpl() const
{
    std::lock_guard<std::mutex> lock(*mutex_);

    apex_assert_hard(message_);

    return message_;
}

void Buffer::write(ConnectionType::ConstPtr message)
{
    std::lock_guard<std::mutex> lock(*mutex_);

    if(!enabled_) {
        return;
    }

    apex_assert_hard(!message_);
    message_ = message;
    if(!message_) {
        throw std::runtime_error("message is empty");
    }
}

void Buffer::disable()
{
    std::lock_guard<std::mutex> lock(*mutex_);

    enabled_ = false;
}

void Buffer::free()
{
    std::lock_guard<std::mutex> lock(*mutex_);

    message_.reset();
}

bool Buffer::isFilled() const
{
    std::lock_guard<std::mutex> lock(*mutex_);

    return (bool) message_;
}
