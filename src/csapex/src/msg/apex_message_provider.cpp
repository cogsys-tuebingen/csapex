/// HEADER
#include <csapex/msg/apex_message_provider.h>

/// COMPONENT
#include <csapex/core/settings.h>
#include <csapex/msg/message_factory.h>

/// SYSTEM
#include <boost/assign.hpp>

using namespace csapex;

boost::shared_ptr<MessageProvider> ApexMessageProvider::make()
{
    return boost::shared_ptr<MessageProvider> (new ApexMessageProvider);
}

void ApexMessageProvider::load(const std::string& file)
{
    file_ = file;
    msg_ = boost::dynamic_pointer_cast<connection_types::Message>(MessageFactory::readMessage(file));

    setSlotCount(1);
}

bool ApexMessageProvider::hasNext()
{
    return (bool) msg_;
}

connection_types::Message::Ptr ApexMessageProvider::next(std::size_t slot)
{
    connection_types::Message::Ptr r = msg_;
    msg_.reset();
    return r;
}

std::string ApexMessageProvider::getLabel(std::size_t slot) const
{
    return msg_->toType()->name();
}

std::vector<std::string> ApexMessageProvider::getExtensions() const
{
    return boost::assign::list_of<std::string> (Settings::message_extension);
}


Memento::Ptr ApexMessageProvider::getState() const
{
    Memento::Ptr r(new Memento);
    return r;
}

void ApexMessageProvider::setParameterState(Memento::Ptr memento)
{

}
