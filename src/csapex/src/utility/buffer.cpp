/// HEADER
#include <csapex/utility/buffer.h>

using namespace csapex;

Buffer::Buffer(std::size_t size)
    : free_(size), used_(0), enabled_(true)
{

}

void Buffer::write(ConnectionType::Ptr message)
{
    free_.acquire();

    if(!enabled_) {
        free_.release();
        used_.release();
        return;
    }

    message_ = message;
    if(!message_) {
        throw std::runtime_error("message is empty");
    }

    used_.release();
}

void Buffer::disable()
{
    enabled_ = false;
    free_.release();
}

void Buffer::free()
{
    free_.release();
}

bool Buffer::isFilled() const
{
    return used_.available();
}
