/// HEADER
#include <csapex/utility/subprocess_channel.h>

/// PROJECT
#include <csapex/utility/assert.h>

/// SYSTEM
#include <iostream>
#include <boost/optional.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

using namespace csapex;
using namespace boost::interprocess;

namespace csapex
{
namespace impl
{
template <typename T>
using ShmAllocator = boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>;

using shared_buffer = typename boost::interprocess::basic_string<uint8_t, std::char_traits<uint8_t>, ShmAllocator<uint8_t>>;

struct ShmBlock
{
    boost::interprocess::interprocess_mutex m;
    boost::interprocess::interprocess_condition message_available;
    boost::interprocess::interprocess_condition message_read;

    SubprocessChannel::MessageType message_type = SubprocessChannel::MessageType::NONE;

    bool is_full = false;
    bool active = true;
};

}  // namespace impl

}  // namespace csapex

// Message

SubprocessChannel::Message::Message(const SubprocessChannel::MessageType type, const std::string& str) : type(type), data(reinterpret_cast<const uint8_t*>(str.c_str())), length(str.size())
{
}
SubprocessChannel::Message::Message(const MessageType type, const uint8_t* data, const std::size_t length) : type(type), data(data), length(length)
{
}

SubprocessChannel::Message::Message(SubprocessChannel::Message&& move)
{
    operator=(std::move(move));
}

SubprocessChannel::Message& SubprocessChannel::Message::operator=(SubprocessChannel::Message&& move)
{
    parent = move.parent;
    move.parent = nullptr;

    type = move.type;
    move.type = MessageType::NONE;
    data = move.data;
    move.data = nullptr;
    length = move.length;
    move.length = 0;

    return *this;
}

SubprocessChannel::Message::~Message()
{
    if (parent) {
        scoped_lock<interprocess_mutex> lock(parent->shm_block_->m);

        parent->shm_segment->destroy<impl::shared_buffer>("data");
        parent->shm_block_->is_full = false;
        parent->is_locked_ = false;
        parent->shm_block_->message_read.notify_all();
    }
}

std::string SubprocessChannel::Message::toString() const
{
    std::stringstream ss;
    const uint8_t* ptr = data;
    for (std::size_t i = 0; i < length; ++i, ++ptr) {
        ss << *ptr;
    }

    return ss.str();
}

SubprocessChannel::Message::Message(SubprocessChannel* parent) : parent(parent)
{
}

// SubprocesChannel

SubprocessChannel::SubprocessChannel(const std::string& name_space, bool is_control_channel, int32_t size)
  : name_space_(std::to_string(getpid()) + "_" + name_space), size_(size), is_control_channel_(is_control_channel), is_locked_(false), is_shutdown_(false)
{
    allocate();
}

SubprocessChannel::~SubprocessChannel()
{
    shared_memory_object::remove(name_space_.c_str());
}

void SubprocessChannel::allocate()
{
    try {
        //        std::cout << "try to create shared memory object " << getUUID() << std::endl;
        shm_segment.reset(new managed_shared_memory(create_only, name_space_.c_str(), size_));

    } catch (const boost::interprocess::interprocess_exception& e) {
        //        std::cout << "could not create shared memory object: " << e.what() << std::endl;
        //        std::cout << "removing shared memory object " << getUUID() << std::endl;
        shared_memory_object::remove(name_space_.c_str());
        shm_segment.reset(new managed_shared_memory(create_only, name_space_.c_str(), size_));
    }

    // Create a managed shared memory segment
    shm_block_ = shm_segment->construct<impl::ShmBlock>("shm")();
}

bool SubprocessChannel::hasMessage() const
{
    std::unique_lock<std::recursive_mutex> channel_lock(channel_mutex_);

    scoped_lock<interprocess_mutex> lock(shm_block_->m);

    return shm_block_->is_full;
}

SubprocessChannel::Message SubprocessChannel::read()
{
    std::unique_lock<std::recursive_mutex> channel_lock(channel_mutex_);

    scoped_lock<interprocess_mutex> lock(shm_block_->m);

    if (!shm_block_->active) {
        throw ShutdownException();
    }

    if (is_shutdown_) {
        throw ShutdownException();
    }

    while (is_locked_) {
        shm_block_->message_read.wait(lock);
    }

    while (!shm_block_->is_full) {
        shm_block_->message_available.wait(lock);

        if (is_shutdown_) {
            throw ShutdownException();
        }
    }

    if (!is_control_channel_) {
        if (shm_block_->message_type == MessageType::SHUTDOWN) {
            throw ShutdownException();
        }
    }

    std::pair<impl::shared_buffer*, managed_shared_memory::size_type> input = shm_segment->find<impl::shared_buffer>("data");

    Message result(this);
    is_locked_ = true;

    if (input.first) {
        result.type = shm_block_->message_type;
        result.data = input.first->c_str();
        result.length = input.first->size();
    }

    return result;
}

void SubprocessChannel::write(const Message& message)
{
    std::unique_lock<std::recursive_mutex> channel_lock(channel_mutex_);

    const impl::ShmAllocator<int> alloc_inst(shm_segment->get_segment_manager());

    scoped_lock<interprocess_mutex> lock(shm_block_->m);

    if (is_shutdown_) {
        return;
    }

    while (shm_block_->is_full) {
        shm_block_->message_read.wait(lock);

        if (is_shutdown_) {
            return;
        }
    }

    apex_assert_hard(!is_locked_);

    shm_segment->construct<impl::shared_buffer>("data")(message.data, message.length, alloc_inst);

    shm_block_->message_type = message.type;
    shm_block_->is_full = true;

    shm_block_->message_available.notify_all();
}

void SubprocessChannel::shutdown()
{
    is_shutdown_ = true;

    scoped_lock<interprocess_mutex> lock(shm_block_->m);
    shm_block_->active = false;
    shm_block_->message_available.notify_all();
}
