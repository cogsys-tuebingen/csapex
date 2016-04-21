#ifndef MESSAGE_RENDERER_H
#define MESSAGE_RENDERER_H

/// PROJECT
#include <csapex/param/param_fwd.h>
#include <csapex/model/model_fwd.h>

/// SYSTEM
#include <memory>
#include <QImage>
#include <stdexcept>
#include <typeindex>

namespace csapex
{

class MessageRenderer
{
public:
    typedef std::shared_ptr<MessageRenderer> Ptr;

public:
    virtual ~MessageRenderer();

    virtual QImage render(const TokenConstPtr& msg) = 0;
    virtual std::type_index messageType() const = 0;

    virtual std::vector<csapex::param::ParameterPtr> getParameters() const
    {
        return std::vector<csapex::param::ParameterPtr>();
    }
};

template <class Message>
class MessageRendererImplementation : public MessageRenderer
{
public:
    virtual QImage render(const TokenConstPtr& msg) final override
    {
        const auto& real_msg = std::dynamic_pointer_cast<Message const>(msg);
        if(real_msg) {
            return doRender(*real_msg);
        } else {
            throw std::runtime_error(std::string("cannot render message of type ") + typeid(Message).name());
        }
    }

    virtual std::type_index messageType() const
    {
        return std::type_index(typeid(Message));
    }

    virtual QImage doRender(const Message& msg) = 0;
};

}

#endif // MESSAGE_RENDERER_H

