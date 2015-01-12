#ifndef BUFFER_H
#define BUFFER_H

/// PROJECT
#include <csapex/msg/message.h>
#include <csapex/utility/assert.h>

class QMutex;

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
    typename boost::shared_ptr<R const> read()
    {
        auto msg = readImpl();

        typename boost::shared_ptr<R const> result = boost::dynamic_pointer_cast<R const> (msg);

        if(result) {
            return result;
        } else {
            std::stringstream e;
            e << "cannot cast message from " << msg->toType()->name() << " to " << type2name(typeid(R));
            throw std::runtime_error(e.str());
        }
    }

    template <typename T>
    bool isType()
    {
        boost::shared_ptr<T const> tmp = boost::dynamic_pointer_cast<T const>(message_);
        return tmp != nullptr;
    }

    void write(ConnectionType::ConstPtr message);

    bool isFilled() const;

private:
    ConnectionType::ConstPtr readImpl() const;

private:
    bool enabled_;

    QMutex* mutex_;
    ConnectionType::ConstPtr message_;
};
}

#endif // BUFFER_H
