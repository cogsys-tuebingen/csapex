/// HEADER
#include <csapex/msg/io.h>

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>

using namespace csapex;

ConnectionTypeConstPtr csapex::msg::getMessage(Input *input)
{
    return input->getMessage();
}

bool csapex::msg::hasMessage(Input *input)
{
    return input->hasMessage();
}
bool csapex::msg::hasMessage(Output *output)
{
    return output->hasMessage();
}

bool csapex::msg::isConnected(Input *input)
{
    return input->isConnected();
}
bool csapex::msg::isConnected(Output *output)
{
    return output->isConnected();
}


void csapex::msg::enable(Input* input)
{
    input->enable();
}

void csapex::msg::disable(Input* input)
{
    input->disable();
}
void csapex::msg::enable(Output* output)
{
    output->enable();
}
void csapex::msg::disable(Output* output)
{
    output->disable();
}

UUID csapex::msg::getUUID(Input *input)
{
    return input->getUUID();
}
UUID csapex::msg::getUUID(Output *output)
{
    return output->getUUID();
}

void csapex::msg::setLabel(Input *input, const std::string &label)
{
    input->setLabel(label);
}

void csapex::msg::setLabel(Output *output, const std::string &label)
{
    output->setLabel(label);
}

void csapex::msg::throwError(const ConnectionTypeConstPtr &msg, const std::type_info &type)
{
    if(!msg) {
        throw std::runtime_error(std::string ("cannot cast null message from to ") + type2name(type));
    } else {
        throw std::runtime_error(std::string ("cannot cast message from ") + msg->toType()->descriptiveName() + " to " + type2name(type));
    }
}

void csapex::msg::publish(Output *output, ConnectionTypeConstPtr message)
{
    output->addMessage(message);
}
