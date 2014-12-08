#ifndef APEX_MESSAGE_PROVIDER_H
#define APEX_MESSAGE_PROVIDER_H

/// COMPONENT
#include <csapex/msg/message_provider.h>

namespace csapex
{

class ApexMessageProvider : public MessageProvider
{
public:
    static boost::shared_ptr<MessageProvider> make();

public:
    void load(const std::string& file);

    virtual bool hasNext();
    virtual connection_types::Message::Ptr next(std::size_t slot);
    virtual std::string getLabel(std::size_t slot) const;

    virtual std::vector<std::string> getExtensions() const;

    virtual Memento::Ptr getState() const;
    virtual void setParameterState(Memento::Ptr memento);

private:
    std::string file_;
    connection_types::Message::Ptr msg_;
};

}

#endif // APEX_MESSAGE_PROVIDER_H
