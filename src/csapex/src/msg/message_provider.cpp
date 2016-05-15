/// HEADER
#include <csapex/msg/message_provider.h>

/// PROJECT
#include <csapex/param/parameter_factory.h>

using namespace csapex;

MessageProvider::MessageProvider()
    : slot_count_(1)
{    
    state.addParameter(csapex::param::ParameterFactory::declareBool("playback/resend", false));
}

MessageProvider::~MessageProvider()
{

}

void MessageProvider::restart()
{

}

TokenData::ConstPtr MessageProvider::getType() const
{
    return type_;
}

void MessageProvider::setType(TokenData::Ptr type)
{
    type_ = type;
}

std::string MessageProvider::getName() const
{
    return name_;
}

void MessageProvider::setName(const std::string& name)
{
    name_ = name;
}

std::vector<csapex::param::Parameter::Ptr> MessageProvider::getParameters() const
{
    return state.getParameters();
}

void MessageProvider::parameterChanged()
{

}

void MessageProvider::setSlotCount(std::size_t slot_count)
{
    if(slot_count != slot_count_) {
        slot_count_ = slot_count;

        slot_count_changed(slot_count_);
    }
}

std::size_t MessageProvider::slotCount() const
{
    return slot_count_;
}

std::string MessageProvider::getLabel(std::size_t /*slot*/) const
{
    return getType()->descriptiveName();
}
