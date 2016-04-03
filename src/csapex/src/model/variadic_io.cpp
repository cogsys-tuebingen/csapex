/// HEADER
#include <csapex/model/variadic_io.h>

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/input.h>
#include <csapex/model/parameterizable.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/io.h>

using namespace csapex;

VariadicInputs::VariadicInputs(ConnectionTypeConstPtr type)
    : type_(type), modifier_(nullptr)
{

}
VariadicInputs::VariadicInputs()
    : VariadicInputs(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

VariadicInputs::~VariadicInputs()
{

}

Connectable* VariadicInputs::createVariadicPort(bool output, ConnectionTypeConstPtr type, const std::string& label, bool optional)
{
    apex_assert_hard(modifier_);
    if(!output) {
        auto result = modifier_->addInput(type, label.empty() ? std::string("Channel") : label, false, optional);
        if(result) {
            count_p_->set((int) modifier_->getMessageInputs().size());
        }
        return result;
    }
    return nullptr;
}

void VariadicInputs::setupParameters(Parameterizable &parameters)
{
    count_p_ = csapex::param::ParameterFactory::declareValue("input count", 2);
    parameters.addParameter(count_p_, [this](param::Parameter* p) {
        updateInputs(p->as<int>());
    });
}

void VariadicInputs::setup(NodeModifier& modifier)
{
    modifier_ = &modifier;
}


void VariadicInputs::updateInputs(int input_count)
{
    apex_assert_hard(modifier_);

    std::vector<Input*> inputs = modifier_->getMessageInputs();
    int current_amount = inputs.size();

    if(current_amount > input_count) {
        for(int i = current_amount; i > input_count ; i--) {
            Input* in = inputs[i - 1];
            if(msg::isConnected(in)) {
                msg::disable(in);
            } else {
                modifier_->removeInput(msg::getUUID(in));
            }
        }
    } else {
        int to_add = input_count - current_amount;
        for(int i = 0 ; i < current_amount; i++) {
            msg::enable(inputs[i]);
        }
        for(int i = 0 ; i < to_add ; i++) {
            createVariadicPort(false, type_, "Channel", true);
        }
    }

}
