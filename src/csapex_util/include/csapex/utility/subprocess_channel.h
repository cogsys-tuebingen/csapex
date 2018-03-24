#ifndef SUBPROCESS_CHANNEL_H
#define SUBPROCESS_CHANNEL_H

/// SYSTEM
#include <mutex>
#include <memory>
#include <boost/interprocess/interprocess_fwd.hpp>

/// FORWARD DECLARATIONS
namespace csapex
{
namespace impl
{
class ShmBlock;
}

class SubprocessChannel
{
public:

    enum class MessageType
    {
        NONE,
        PROCESS_SYNC,
        PROCESS_ASYNC,
        PROCESS_SLOT,
        PROCESS_FINISHED,

        PARAMETER_UPDATE,
        PORT_ADD,
        NODE_STATE_CHANGED,

        SHUTDOWN,
        CHILD_SIGNAL,
        CHILD_ERROR,
        CHILD_EXIT
    };

    class ShutdownException : public std::runtime_error
    {
    public:
        ShutdownException()
            : std::runtime_error("shutdown")
        {}
    };

    struct Message
    {
        Message() = default;
        Message(const MessageType type, const std::string& str);
        Message(const MessageType type, const uint8_t* data, const std::size_t length);

        Message(const Message& copy) = delete;
        Message(Message&& move);

        Message& operator = (const Message& copy) = delete;
        Message& operator = (Message&& move);

        ~Message();

        MessageType type = MessageType::NONE;
        const uint8_t* data = nullptr;
        std::size_t length = 0;

        std::string toString() const;

    protected:
        friend class SubprocessChannel;
        Message(SubprocessChannel* parent);

        SubprocessChannel* parent = nullptr;
    };

public:
    SubprocessChannel(const std::string& name_space, bool is_control_channel = false, int32_t size = -1);

    ~SubprocessChannel();

    Message read();
    void write(const Message& message);
    bool hasMessage() const;


    void shutdown();

private:
    void allocate();

private:
    std::shared_ptr<boost::interprocess::managed_shared_memory> shm_segment;

    mutable std::recursive_mutex channel_mutex_;

    std::string name_space_;

    int32_t size_;
    bool is_control_channel_;

    bool is_locked_;
    bool is_shutdown_;

    impl::ShmBlock* shm_block_;
};

}

#endif // SUBPROCESS_CHANNEL_H
