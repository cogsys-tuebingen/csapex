/// HEADER
#include <csapex/utility/buffer.h>

using namespace csapex;

Buffer::Buffer(std::size_t size)
    : free_(size), used_(0)
{

}

void Buffer::write(ConnectionType::Ptr message)
{
    free_.acquire();

    message_ = message;
    if(!message_) {
        throw std::runtime_error("message is empty");
    }

    used_.release();
}

void Buffer::waitForMessage()
{
    used_.acquire();
}

void Buffer::waitForEmpty()
{
    free_.acquire();
}

void Buffer::free()
{
    free_.release();
}

bool Buffer::isFilled() const
{
    return used_.available();
}
