/// HEADER
#include <csapex/msg/io.h>

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/token.h>

using namespace csapex;

TokenDataConstPtr csapex::msg::getMessage(Input *input)
{
    apex_assert_hard_msg(input->isEnabled(), "you have requested a message from a disabled input");
    return input->getToken()->getTokenData();
}

bool csapex::msg::hasMessage(Input *input)
{
    return input->hasMessage() && input->isEnabled();
}
bool csapex::msg::hasMessage(Output *output)
{
    return output->hasMessage() && output->isEnabled();
}

bool csapex::msg::isEnabled(Input *input)
{
    return input->isEnabled();
}
bool csapex::msg::isEnabled(Output *output)
{
    return output->isEnabled();
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

void csapex::msg::throwError(const TokenDataConstPtr &msg, const std::type_info &type)
{
    if(!msg) {
        throw std::runtime_error(std::string ("cannot cast null message from to ") + type2name(type));
    } else {
        throw std::runtime_error(std::string ("cannot cast message from ") + msg->toType()->descriptiveName() + " to " + type2name(type));
    }
}

void csapex::msg::publish(Output *output, TokenDataConstPtr message)
{
    output->addMessage(std::make_shared<Token>(message));
}
