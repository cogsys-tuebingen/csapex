#ifndef BUFFER_H
#define BUFFER_H

/// PROJECT
#include <csapex/msg/message.h>
#include <csapex/utility/assert.h>

namespace csapex
{
class Buffer
{
public:
    Buffer();
    ~Buffer();

    void disable();
    void free();

    template <typename R>
    typename std::shared_ptr<R const> read()
    {
        auto msg = readImpl();

        typename std::shared_ptr<R const> result = std::dynamic_pointer_cast<R const> (msg);

        if(result) {
            return result;
        } else {
            throw std::runtime_error(std::string ("cannot cast message from ") + msg->toType()->name() + " to " + type2name(typeid(R)));
        }
    }

    template <typename T>
    bool isType()
    {
        std::shared_ptr<T const> tmp = std::dynamic_pointer_cast<T const>(message_);
        return tmp != nullptr;
    }

    void write(ConnectionType::ConstPtr message);

    bool isFilled() const;

private:
    ConnectionType::ConstPtr readImpl() const;

private:
    bool enabled_;

    std::mutex* mutex_;
    ConnectionType::ConstPtr message_;
};
}

#endif // BUFFER_H
