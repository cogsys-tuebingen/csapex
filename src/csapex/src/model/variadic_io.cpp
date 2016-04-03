/// HEADER
#include <csapex/model/variadic_io.h>

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/parameterizable.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/io.h>

using namespace csapex;

VariadicBase::VariadicBase(ConnectionTypeConstPtr type)
    : variadic_type_(type), variadic_modifier_(nullptr)
{

}
VariadicBase::VariadicBase()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

VariadicBase::~VariadicBase()
{

}

void VariadicBase::setupVariadic(NodeModifier& modifier)
{
    variadic_modifier_ = &modifier;
}

void VariadicBase::portCountChanged()
{

}



VariadicInputs::VariadicInputs(ConnectionTypeConstPtr type)
    : VariadicBase(type)
{

}
VariadicInputs::VariadicInputs()
    : VariadicInputs(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicInputs::createVariadicPort(bool output, ConnectionTypeConstPtr type, const std::string& label, bool optional)
{
    apex_assert_hard(variadic_modifier_);
    apex_assert_hard(!output);
    auto result = variadic_modifier_->addInput(type, label.empty() ? std::string("Channel") : label, false, optional);
    if(result) {
        input_count_->set((int) variadic_modifier_->getMessageInputs().size());
    }
    return result;
}

void VariadicInputs::setupVariadicParameters(Parameterizable &parameters)
{
    input_count_ = csapex::param::ParameterFactory::declareValue("input count", 2);
    parameters.addParameter(input_count_, [this](param::Parameter* p) {
        updateInputs(p->as<int>());
    });
}


void VariadicInputs::updateInputs(int count)
{
    apex_assert_hard(variadic_modifier_);

    std::vector<Input*> inputs = variadic_modifier_->getMessageInputs();
    int current_amount = inputs.size();

    if(current_amount > count) {
        bool connected = false;
        for(int i = current_amount; i > count ; i--) {
            Input* in = inputs[i - 1];
            if(connected || msg::isConnected(in)) {
                msg::disable(in);
                connected = true;
            } else {
                variadic_modifier_->removeInput(msg::getUUID(in));
            }
        }
    } else {
        int to_add = count - current_amount;
        for(int i = 0 ; i < current_amount; i++) {
            msg::enable(inputs[i]);
        }
        for(int i = 0 ; i < to_add ; i++) {
            createVariadicPort(false, variadic_type_, "Channel", true);
        }
    }

    portCountChanged();
}



VariadicOutputs::VariadicOutputs(ConnectionTypeConstPtr type)
    : VariadicBase(type)
{

}
VariadicOutputs::VariadicOutputs()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicOutputs::createVariadicPort(bool output, ConnectionTypeConstPtr type, const std::string& label, bool optional)
{
    apex_assert_hard(variadic_modifier_);
    apex_assert_hard(output);
    auto result = variadic_modifier_->addOutput(type, label.empty() ? std::string("Channel") : label, false);
    if(result) {
        output_count_->set((int) variadic_modifier_->getMessageOutputs().size());
    }
    return result;
}

void VariadicOutputs::setupVariadicParameters(Parameterizable &parameters)
{
    output_count_ = csapex::param::ParameterFactory::declareValue("output count", 2);
    parameters.addParameter(output_count_, [this](param::Parameter* p) {
        updateOutputs(p->as<int>());
    });
}

void VariadicOutputs::updateOutputs(int count)
{
    apex_assert_hard(variadic_modifier_);

    std::vector<Output*> outputs = variadic_modifier_->getMessageOutputs();
    int current_amount = outputs.size();

    if(current_amount > count) {
        bool connected = false;
        for(int i = current_amount; i > count ; i--) {
            Output* out = outputs[i - 1];
            if(connected || msg::isConnected(out)) {
                msg::disable(out);
                connected = true;
            } else {
                variadic_modifier_->removeOutput(msg::getUUID(out));
            }
        }
    } else {
        int to_add = count - current_amount;
        for(int i = 0 ; i < current_amount; i++) {
            msg::enable(outputs[i]);
        }
        for(int i = 0 ; i < to_add ; i++) {
            createVariadicPort(true, variadic_type_, "Channel", true);
        }
    }

    portCountChanged();
}






VariadicIO::VariadicIO(ConnectionTypeConstPtr type)
    : VariadicBase(type), VariadicInputs(type), VariadicOutputs(type)
{

}
VariadicIO::VariadicIO()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicIO::createVariadicPort(bool output, ConnectionTypeConstPtr type, const std::string& label, bool optional)
{
    apex_assert_hard(variadic_modifier_);
    if(output) {
        return VariadicOutputs::createVariadicPort(output, type, label, optional);
    } else {
        return VariadicInputs::createVariadicPort(output, type, label, optional);
    }
}

void VariadicIO::setupVariadicParameters(Parameterizable &parameters)
{
    VariadicInputs::setupVariadicParameters(parameters);
    VariadicOutputs::setupVariadicParameters(parameters);
}
