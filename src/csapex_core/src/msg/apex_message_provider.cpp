/// HEADER
#include <csapex/msg/apex_message_provider.h>

/// COMPONENT
#include <csapex/core/settings.h>
#include <csapex/factory/message_factory.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

std::shared_ptr<MessageProvider> ApexMessageProvider::make()
{
    return std::shared_ptr<MessageProvider>(new ApexMessageProvider);
}

void ApexMessageProvider::load(const std::string& file)
{
    file_ = file;

    msg_ = std::dynamic_pointer_cast<connection_types::Message>(MessageFactory::readFile(file));

    setSlotCount(1);
}

bool ApexMessageProvider::hasNext()
{
    return (bool)msg_ || state.readParameter<bool>("playback/resend");
}

connection_types::Message::Ptr ApexMessageProvider::next(std::size_t /*slot*/)
{
    connection_types::Message::Ptr r;
    std::swap(r, msg_);
    return r;
}

std::string ApexMessageProvider::getLabel(std::size_t /*slot*/) const
{
    return msg_->toType()->descriptiveName();
}

std::vector<std::string> ApexMessageProvider::getExtensions() const
{
    return { Settings::message_extension, Settings::message_extension_compressed, Settings::message_extension_binary };
}

GenericStatePtr ApexMessageProvider::getState() const
{
    GenericStatePtr r(new GenericState);
    return r;
}

void ApexMessageProvider::setParameterState(GenericStatePtr /*memento*/)
{
}
