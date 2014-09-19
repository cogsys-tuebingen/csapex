#ifndef BUFFER_H
#define BUFFER_H

/// PROJECT
#include <csapex/msg/message.h>

/// SYSTEM
#include <QSemaphore>

namespace csapex
{
class Buffer
{
public:
    Buffer(std::size_t size);

    void disable();
    void free();

    template <typename R>
    typename boost::shared_ptr<R> read()
    {
        used_.acquire();

        typename R::Ptr result = boost::dynamic_pointer_cast<R> (message_);

        if(result) {
            return result;
        } else {
            std::stringstream e;
            e << "cannot cast message from " << message_->toType()->name() << " to " << type2name(typeid(R));
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
    QSemaphore free_;
    QSemaphore used_;

    bool enabled_;

    ConnectionType::Ptr message_;
};
}

#endif // BUFFER_H
