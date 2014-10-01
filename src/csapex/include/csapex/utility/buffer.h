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

    void disable();
    void free();

    template <typename R>
    typename boost::shared_ptr<R> read()
    {
        ConnectionType::Ptr msg = readImpl();

        typename R::Ptr result = boost::dynamic_pointer_cast<R> (msg);

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
        boost::shared_ptr<T> tmp = boost::dynamic_pointer_cast<T>(message_);
        return tmp;
    }

    void write(ConnectionType::Ptr message);

    bool isFilled() const;

private:
    ConnectionType::Ptr readImpl() const;

private:
    bool enabled_;

    QMutex* mutex_;
    ConnectionType::Ptr message_;
};
}

#endif // BUFFER_H
