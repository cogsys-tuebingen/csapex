#ifndef MESSAGE_RENDERER_H
#define MESSAGE_RENDERER_H

/// PROJECT
#include <csapex/csapex_fwd.h>
#include <utils_param/param_fwd.h>

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

public:
    virtual ~MessageRenderer();

    virtual QSharedPointer<QImage> render(const ConnectionTypeConstPtr& msg) = 0;
    virtual const std::type_info* messageType() const = 0;

    virtual std::vector<param::ParameterPtr> getParameters() const
    {
        return std::vector<param::ParameterPtr>();
    }
};

template <class Message>
class MessageRendererImplementation : public MessageRenderer
{
public:
    virtual QSharedPointer<QImage> render(const ConnectionTypeConstPtr& msg) final override
    {
        const auto& real_msg = boost::dynamic_pointer_cast<Message const>(msg);
        if(real_msg) {
            return doRender(*real_msg);
        } else {
            throw std::runtime_error(std::string("cannot render message of type ") + typeid(*msg).name());
        }
    }

    virtual const std::type_info* messageType() const
    {
        return &typeid(Message);
    }

    virtual QSharedPointer<QImage> doRender(const Message& msg) = 0;
};

}

#endif // MESSAGE_RENDERER_H

