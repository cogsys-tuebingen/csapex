#ifndef APEX_MESSAGE_PROVIDER_H
#define APEX_MESSAGE_PROVIDER_H

/// COMPONENT
#include <csapex/msg/message_provider.h>
#include <csapex/csapex_export.h>

namespace csapex
{

class CSAPEX_EXPORT ApexMessageProvider : public MessageProvider
{
public:
    static std::shared_ptr<MessageProvider> make();

public:
    void load(const std::string& file) override;

    virtual bool hasNext() override;
    virtual connection_types::Message::Ptr next(std::size_t slot) override;
    virtual std::string getLabel(std::size_t slot) const override;

    virtual std::vector<std::string> getExtensions() const override;

    virtual Memento::Ptr getState() const override;
    virtual void setParameterState(Memento::Ptr memento) override;

private:
    std::string file_;
    connection_types::Message::Ptr msg_;
    connection_types::Message::Ptr cache_msg_;
};

}

#endif // APEX_MESSAGE_PROVIDER_H
