/// HEADER
#include <csapex/utility/buffer.h>

/// SYSTEM
#include <QMutex>

using namespace csapex;

Buffer::Buffer()
    : enabled_(true)
{

}

void Buffer::write(ConnectionType::Ptr message)
{
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
    enabled_ = false;
}

void Buffer::free()
{
    message_.reset();
}

bool Buffer::isFilled() const
{
    return message_;
}
