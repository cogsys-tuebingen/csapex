/// HEADER
#include <csapex/msg/message_allocator.h>

using namespace csapex;

MessageAllocator::MessageAllocator()
    : allocator_(nullptr)
{

}

MessageAllocator::~MessageAllocator()
{
    delete allocator_;
}
