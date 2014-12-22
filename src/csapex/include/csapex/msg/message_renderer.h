#ifndef MESSAGE_RENDERER_H
#define MESSAGE_RENDERER_H

/// PROJECT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <QSharedPointer>
#include <QImage>
#include <stdexcept>

namespace csapex
{

class MessageRenderer
{
public:
    typedef boost::shared_ptr<MessageRenderer> Ptr;

    virtual QSharedPointer<QImage> render(const ConnectionTypePtr& msg) = 0;
    virtual const std::type_info* messageType() = 0;

public:
    virtual ~MessageRenderer();
};

template <class Message>
class MessageRendererImplementation : public MessageRenderer
{
public:
    virtual QSharedPointer<QImage> render(const ConnectionTypePtr& msg)
    {
        const boost::shared_ptr<Message>& real_msg = boost::dynamic_pointer_cast<Message>(msg);
        if(real_msg) {
            return doRender(*real_msg);
        } else {
            throw std::runtime_error(std::string("cannot render message of type ") + typeid(*msg).name());
        }
    }

    virtual const std::type_info* messageType()
    {
        return &typeid(Message);
    }

    virtual QSharedPointer<QImage> doRender(const Message& msg) = 0;
};

}

#endif // MESSAGE_RENDERER_H

