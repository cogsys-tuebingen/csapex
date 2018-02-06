#ifndef SUBPROCESS_CHANNEL_H
#define SUBPROCESS_CHANNEL_H

/// SYSTEM
#include <boost/optional.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <mutex>

#include <iostream>

namespace csapex
{
template <typename T>
using ShmAllocator = boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>;


class SubprocessChannel
{
public:
    using shared_buffer = typename boost::interprocess::basic_string<uint8_t, std::char_traits<uint8_t>, ShmAllocator<uint8_t>>;

    enum class MessageType
    {
        NONE,
        PROCESS,
        PARAMETER_UPDATE,

        SHUTDOWN,
        CHILD_EXIT
    };

    struct Message
    {
        Message() = default;
        Message(const MessageType type, const std::string& str)
            : type(type), data(reinterpret_cast<const uint8_t*>(str.c_str())), length(str.size())
        {}
        Message(const MessageType type, const uint8_t* data, const std::size_t length)
            : type(type), data(data), length(length)
        {}

        Message(const Message& copy) = delete;
        Message(Message&& move)
        {
            operator = (std::move(move));
        }

        Message& operator = (const Message& copy) = delete;
        Message& operator = (Message&& move)
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

        ~Message()
        {
            if(parent) {
                parent->shm_segment->destroy<shared_buffer>("data");
                parent->shm_block_->is_full = false;
                parent->is_locked_ = false;
                parent->shm_block_->message_read.notify_all();
            }
        }

        MessageType type = MessageType::NONE;
        const uint8_t* data = nullptr;
        std::size_t length = 0;

        std::string toString() const
        {
            std::stringstream ss;
            const uint8_t* ptr = data;
            for(std::size_t i = 0; i < length; ++i, ++ptr) {
                ss << *ptr;
            }

            return ss.str();
        }

    protected:
        friend class SubprocessChannel;
        Message(SubprocessChannel* parent)
            : parent(parent)
        {

        }

        SubprocessChannel* parent = nullptr;
    };

public:
    SubprocessChannel(const std::string& name_space, bool is_control_channel = false, int32_t size = -1)
        : name_space_(name_space), size_(size), is_control_channel_(is_control_channel),
          is_locked_(false)
    {
        allocate();
    }

    ~SubprocessChannel()
    {
        using namespace boost::interprocess;
        shared_memory_object::remove(name_space_.c_str());
    }

    void allocate()
    {
        using namespace boost::interprocess;
        try {
            //        std::cout << "try to create shared memory object " << getUUID() << std::endl;
            shm_segment.reset(new managed_shared_memory(create_only, name_space_.c_str(), size_));

        } catch(const boost::interprocess::interprocess_exception& e) {
            //        std::cout << "could not create shared memory object: " << e.what() << std::endl;
            //        std::cout << "removing shared memory object " << getUUID() << std::endl;
            shared_memory_object::remove(name_space_.c_str());
            shm_segment.reset(new managed_shared_memory(create_only, name_space_.c_str(), size_));
        }


        //Create a managed shared memory segment
        shm_block_ = shm_segment->construct<ShmBlock>("shm")();
    }


    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock()
    {
        using namespace boost::interprocess;
        scoped_lock<interprocess_mutex> lock(shm_block_->m);
        return lock;
    }

    bool hasMessage() const
    {
        std::unique_lock<std::recursive_mutex> channel_lock(channel_mutex_);

        using namespace boost::interprocess;

        scoped_lock<interprocess_mutex> lock(shm_block_->m);

        return shm_block_->is_full;
    }

    Message read()
    {
        using namespace boost::interprocess;

        std::unique_lock<std::recursive_mutex> channel_lock(channel_mutex_);

        scoped_lock<interprocess_mutex> lock(shm_block_->m);

        while(is_locked_) {
            shm_block_->message_read.wait(lock);
        }

        while(!shm_block_->is_full) {
            shm_block_->message_available.wait(lock);
        }

        if(!is_control_channel_) {
            if(shm_block_->message_type == MessageType::SHUTDOWN) {
                throw std::runtime_error("shutdown");
            }
        }

        std::pair<shared_buffer*, managed_shared_memory::size_type> input
                = shm_segment->find<shared_buffer> ("data");

        Message result(this);
        is_locked_ = true;

        if(input.first) {
            result.type = shm_block_->message_type;
            result.data = input.first->c_str();
            result.length = input.first->size();
        }

        return result;
    }

    void write(const Message& message)
    {
        using namespace boost::interprocess;

        std::unique_lock<std::recursive_mutex> channel_lock(channel_mutex_);

        const ShmAllocator<int> alloc_inst (shm_segment->get_segment_manager());

        scoped_lock<interprocess_mutex> lock(shm_block_->m);

        while(shm_block_->is_full) {
            shm_block_->message_read.wait(lock);
        }

        shm_segment->construct<shared_buffer>
                ("data")
                (message.data, message.length, alloc_inst);

        shm_block_->message_type = message.type;
        shm_block_->is_full = true;

        shm_block_->message_available.notify_all();
    }

    void shutdown()
    {
        using namespace boost::interprocess;

        if(channel_mutex_.try_lock()) {
            // nothing blocked, so nothing to do
        } else {
            scoped_lock<interprocess_mutex> lock(shm_block_->m);

            shm_block_->is_full = true;
            shm_block_->message_type = MessageType::SHUTDOWN;
            shm_block_->message_available.notify_all();
        }
    }

private:
    std::shared_ptr<boost::interprocess::managed_shared_memory> shm_segment;

    mutable std::recursive_mutex channel_mutex_;

    std::string name_space_;

    int32_t size_;
    bool is_control_channel_;

    bool is_locked_;


    struct ShmBlock
    {
        ShmBlock()
            : message_type(MessageType::NONE),
              is_full(false),
              active(true)
        {
        }

        boost::interprocess::interprocess_mutex m;
        boost::interprocess::interprocess_condition message_available;
        boost::interprocess::interprocess_condition message_read;


        MessageType message_type;

        bool is_full;
        bool active;
    };

    ShmBlock* shm_block_;
};

}

#endif // SUBPROCESS_CHANNEL_H
