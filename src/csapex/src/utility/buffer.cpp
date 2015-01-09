/// HEADER
#include <csapex/utility/buffer.h>

/// SYSTEM
#include <QMutex>

using namespace csapex;

Buffer::Buffer()
    : enabled_(true), mutex_(new QMutex)
{

}

Buffer::~Buffer()
{
    delete mutex_;
}

ConnectionType::Ptr Buffer::readImpl() const
{
    QMutexLocker lock(mutex_);

    apex_assert_hard(message_);

    return message_;
}

void Buffer::write(ConnectionType::Ptr message)
{
    QMutexLocker lock(mutex_);

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
    QMutexLocker lock(mutex_);

    enabled_ = false;
}

void Buffer::free()
{
    QMutexLocker lock(mutex_);

    message_.reset();
}

bool Buffer::isFilled() const
{
    QMutexLocker lock(mutex_);

    return (bool) message_;
}
