/// HEADER
#include <csapex/model/variadic_io.h>

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/trigger.h>
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


///
/// INPUTS
///

VariadicInputs::VariadicInputs(ConnectionTypeConstPtr type)
    : VariadicBase(type)
{

}
VariadicInputs::VariadicInputs()
    : VariadicInputs(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicInputs::createVariadicPort(ConnectorType port_type, ConnectionTypeConstPtr type, const std::string &label, bool optional)
{
    apex_assert_hard(port_type == ConnectorType::INPUT);
    return createVariadicInput(type, label, optional);
}

Connectable* VariadicInputs::createVariadicInput(ConnectionTypeConstPtr type, const std::string& label, bool optional)
{
    apex_assert_hard(variadic_modifier_);
    auto result = variadic_modifier_->addInput(type, label.empty() ? std::string("Channel") : label, false, optional);
    if(result) {
        variadic_inputs_.emplace_back(result);
        input_count_->set((int) variadic_inputs_.size());
    }
    return result;
}

void VariadicInputs::setupVariadicParameters(Parameterizable &parameters)
{
    input_count_ = csapex::param::ParameterFactory::declareValue("input count", 0);
    parameters.addParameter(input_count_, [this](param::Parameter* p) {
        updateInputs(p->as<int>());
    });
}


void VariadicInputs::updateInputs(int count)
{
    apex_assert_hard(variadic_modifier_);

    int current_amount = variadic_inputs_.size();

    if(current_amount > count) {
        bool connected = false;
        for(int i = current_amount; i > count ; i--) {
            Input* in = variadic_inputs_[i - 1];
            if(connected || msg::isConnected(in)) {
                msg::disable(in);
                connected = true;
            } else {
                variadic_modifier_->removeInput(msg::getUUID(in));
                variadic_inputs_.erase(std::find(variadic_inputs_.begin(), variadic_inputs_.end(), in));
            }
        }
    } else {
        int to_add = count - current_amount;
        for(int i = 0 ; i < current_amount; i++) {
            msg::enable(variadic_inputs_[i]);
        }
        for(int i = 0 ; i < to_add ; i++) {
            createVariadicInput(variadic_type_, "Channel", true);
        }
    }

    portCountChanged();
}


///
/// OUTPUTS
///


VariadicOutputs::VariadicOutputs(ConnectionTypeConstPtr type)
    : VariadicBase(type)
{

}
VariadicOutputs::VariadicOutputs()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicOutputs::createVariadicPort(ConnectorType port_type, ConnectionTypeConstPtr type, const std::string &label, bool optional)
{
    apex_assert_hard(port_type == ConnectorType::OUTPUT);
    return createVariadicOutput(type, label);
}

Connectable* VariadicOutputs::createVariadicOutput(ConnectionTypeConstPtr type, const std::string& label)
{
    apex_assert_hard(variadic_modifier_);
    auto result = variadic_modifier_->addOutput(type, label.empty() ? std::string("Channel") : label, false);
    if(result) {
        variadic_outputs_.emplace_back(result);
        output_count_->set((int) variadic_modifier_->getMessageOutputs().size());
    }
    return result;
}

void VariadicOutputs::setupVariadicParameters(Parameterizable &parameters)
{
    output_count_ = csapex::param::ParameterFactory::declareValue("output count", 0);
    parameters.addParameter(output_count_, [this](param::Parameter* p) {
        updateOutputs(p->as<int>());
    });
}

void VariadicOutputs::updateOutputs(int count)
{
    apex_assert_hard(variadic_modifier_);

    int current_amount = variadic_outputs_.size();

    if(current_amount > count) {
        bool connected = false;
        for(int i = current_amount; i > count ; i--) {
            Output* out = variadic_outputs_[i - 1];
            if(connected || msg::isConnected(out)) {
                msg::disable(out);
                connected = true;
            } else {
                variadic_modifier_->removeOutput(msg::getUUID(out));
                variadic_outputs_.erase(std::find(variadic_outputs_.begin(), variadic_outputs_.end(), out));
            }
        }
    } else {
        int to_add = count - current_amount;
        for(int i = 0 ; i < current_amount; i++) {
            msg::enable(variadic_outputs_[i]);
        }
        for(int i = 0 ; i < to_add ; i++) {
            createVariadicOutput(variadic_type_, "Channel");
        }
    }

    portCountChanged();
}


///
/// TRIGGERS
///


VariadicTriggers::VariadicTriggers(ConnectionTypeConstPtr type)
    : VariadicBase(type)
{

}
VariadicTriggers::VariadicTriggers()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicTriggers::createVariadicPort(ConnectorType port_type, ConnectionTypeConstPtr type, const std::string &label, bool optional)
{
    apex_assert_hard(port_type == ConnectorType::TRIGGER);
    return createVariadicTrigger(label);
}


Connectable* VariadicTriggers::createVariadicTrigger(const std::string& label)
{
    apex_assert_hard(variadic_modifier_);
    auto result = variadic_modifier_->addTrigger(label.empty() ? std::string("Trigger") : label);
    if(result) {
        variadic_triggers_.emplace_back(result);
        trigger_count_->set((int) variadic_modifier_->getTriggers().size());
    }
    return result;
}

void VariadicTriggers::setupVariadicParameters(Parameterizable &parameters)
{
    trigger_count_ = csapex::param::ParameterFactory::declareValue("trigger count", 0);
    parameters.addParameter(trigger_count_, [this](param::Parameter* p) {
        updateTriggers(p->as<int>());
    });
}

void VariadicTriggers::updateTriggers(int count)
{
    apex_assert_hard(variadic_modifier_);

    int current_amount = variadic_triggers_.size();

    if(current_amount > count) {
        bool connected = false;
        for(int i = current_amount; i > count ; i--) {
            Trigger* t = variadic_triggers_[i - 1];
            if(connected || t->isConnected()) {
                t->disable();
                connected = true;
            } else {
                variadic_modifier_->removeTrigger(t->getUUID());
                variadic_triggers_.erase(std::find(variadic_triggers_.begin(), variadic_triggers_.end(), t));
            }
        }
    } else {
        int to_add = count - current_amount;
        for(int i = 0 ; i < current_amount; i++) {
            variadic_triggers_[i]->enable();
        }
        for(int i = 0 ; i < to_add ; i++) {
            createVariadicTrigger("Trigger");
        }
    }

    portCountChanged();
}



///
/// SLOTS
///


VariadicSlots::VariadicSlots(ConnectionTypeConstPtr type)
    : VariadicBase(type)
{

}
VariadicSlots::VariadicSlots()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicSlots::createVariadicSlot(const std::string& label)
{
    apex_assert_hard(variadic_modifier_);
    auto cb = [](){

    };

    auto result = variadic_modifier_->addSlot(label.empty() ? std::string("Trigger") : label, cb);
    if(result) {
        variadic_slots_.emplace_back(result);
        slot_count_->set((int) variadic_modifier_->getSlots().size());
    }
    return result;
}

Connectable* VariadicSlots::createVariadicPort(ConnectorType port_type, ConnectionTypeConstPtr type, const std::string &label, bool optional)
{
    apex_assert_hard(port_type == ConnectorType::SLOT_T);
    return createVariadicSlot(label);
}

void VariadicSlots::setupVariadicParameters(Parameterizable &parameters)
{
    slot_count_ = csapex::param::ParameterFactory::declareValue("slot count", 0);
    parameters.addParameter(slot_count_, [this](param::Parameter* p) {
        updateSlots(p->as<int>());
    });
}

void VariadicSlots::updateSlots(int count)
{
    apex_assert_hard(variadic_modifier_);

    int current_amount = variadic_slots_.size();

    if(current_amount > count) {
        bool connected = false;
        for(int i = current_amount; i > count ; i--) {
            Slot* s = variadic_slots_[i - 1];
            if(connected || s->isConnected()) {
                s->disable();
                connected = true;
            } else {
                variadic_modifier_->removeSlot(s->getUUID());
                variadic_slots_.erase(std::find(variadic_slots_.begin(), variadic_slots_.end(), s));
            }
        }
    } else {
        int to_add = count - current_amount;
        for(int i = 0 ; i < current_amount; i++) {
            variadic_slots_[i]->enable();
        }
        for(int i = 0 ; i < to_add ; i++) {
            createVariadicSlot("Slot");
        }
    }

    portCountChanged();
}





///
/// IO
///


VariadicIO::VariadicIO(ConnectionTypeConstPtr type)
    : VariadicBase(type), VariadicInputs(type), VariadicOutputs(type)
{

}
VariadicIO::VariadicIO()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicIO::createVariadicPort(ConnectorType port_type, ConnectionTypeConstPtr type, const std::string& label, bool optional)
{
    apex_assert_hard(variadic_modifier_);
    switch (port_type) {
    case ConnectorType::OUTPUT:
        return createVariadicOutput(type, label);
    case ConnectorType::INPUT:
        return createVariadicInput(type, label, optional);
    default:
        throw std::logic_error(std::string("Variadic port of type ") + port_type::name(port_type) + " is not supported.");
    }
}

void VariadicIO::setupVariadicParameters(Parameterizable &parameters)
{
    VariadicInputs::setupVariadicParameters(parameters);
    VariadicOutputs::setupVariadicParameters(parameters);
}




///
/// VARIADIC
///


Variadic::Variadic(ConnectionTypeConstPtr type)
    : VariadicBase(type), VariadicInputs(type), VariadicOutputs(type), VariadicTriggers(type), VariadicSlots(type)
{

}
Variadic::Variadic()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* Variadic::createVariadicPort(ConnectorType port_type, ConnectionTypeConstPtr type, const std::string& label, bool optional)
{
    apex_assert_hard(variadic_modifier_);
    switch (port_type) {
    case ConnectorType::OUTPUT:
        return createVariadicOutput(type, label);
    case ConnectorType::INPUT:
        return createVariadicInput(type, label, optional);
    case ConnectorType::SLOT_T:
        return createVariadicSlot(label);
    case ConnectorType::TRIGGER:
        return createVariadicTrigger(label);
    default:
        throw std::logic_error(std::string("Variadic port of type ") + port_type::name(port_type) + " is not supported.");
    }
}

void Variadic::setupVariadicParameters(Parameterizable &parameters)
{
    VariadicInputs::setupVariadicParameters(parameters);
    VariadicOutputs::setupVariadicParameters(parameters);
    VariadicTriggers::setupVariadicParameters(parameters);
    VariadicSlots::setupVariadicParameters(parameters);
}
