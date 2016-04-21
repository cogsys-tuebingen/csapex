/// HEADER
#include <csapex/model/variadic_io.h>

/// PROJECT
#include <csapex/model/node_modifier.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/event.h>
#include <csapex/model/parameterizable.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/io.h>

using namespace csapex;

VariadicBase::VariadicBase(TokenConstPtr type)
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

VariadicInputs::VariadicInputs(TokenConstPtr type)
    : VariadicBase(type)
{

}
VariadicInputs::VariadicInputs()
    : VariadicInputs(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicInputs::createVariadicPort(ConnectorType port_type, TokenConstPtr type, const std::string &label, bool optional)
{
    apex_assert_hard(port_type == ConnectorType::INPUT);
    return createVariadicInput(type, label, optional);
}

int VariadicInputs::getVariadicInputCount() const
{
    return variadic_inputs_.size();
}

void VariadicInputs::removeVariadicInput(InputPtr input)
{
    variadic_modifier_->removeInput(input->getUUID());
    variadic_inputs_.erase(std::find(variadic_inputs_.begin(), variadic_inputs_.end(), input));
}

void VariadicInputs::removeVariadicInputById(const UUID& input)
{
    for(auto it = variadic_inputs_.begin(); it != variadic_inputs_.end(); ) {
        InputPtr i = *it;
        if(i->getUUID() == input) {
            removeVariadicInput(i);
            return;
        } else {
            ++it;
        }
    }
}

Input *VariadicInputs::createVariadicInput(TokenConstPtr type, const std::string& label, bool optional)
{
    apex_assert_hard(variadic_modifier_);
    Input* result = variadic_modifier_->addInput(type, label.empty() ? std::string("Channel") : label, false, optional);
    if(result) {
        variadic_inputs_.push_back(std::dynamic_pointer_cast<Input>(result->shared_from_this()));
        input_count_->set((int) variadic_inputs_.size());
    }
    return result;
}

void VariadicInputs::setupVariadicParameters(Parameterizable &parameters)
{
    input_count_ = csapex::param::ParameterFactory::declareValue("input count", 0);
    parameters.addParameter(input_count_, [this](param::Parameter* p) {
        int count = p->as<int>();
        if(count >= 0) {
            updateInputs(p->as<int>());
        } else {
            p->set(0);
        }
    });
}


void VariadicInputs::updateInputs(int count)
{
    if(count < 0) {
        return;
    }

    apex_assert_hard(variadic_modifier_);

    int current_amount = variadic_inputs_.size();

    if(current_amount > count) {
        bool connected = false;
        for(int i = current_amount; i > count ; i--) {
            InputPtr in = variadic_inputs_.at(i - 1);
            if(connected || in->isConnected()) {
                in->disable();
                connected = true;
            } else {
                removeVariadicInput(in);
            }
        }
    } else {
        int to_add = count - current_amount;
        for(int i = 0 ; i < current_amount; i++) {
            variadic_inputs_.at(i)->enable();
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


VariadicOutputs::VariadicOutputs(TokenConstPtr type)
    : VariadicBase(type)
{

}
VariadicOutputs::VariadicOutputs()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicOutputs::createVariadicPort(ConnectorType port_type, TokenConstPtr type, const std::string &label, bool optional)
{
    apex_assert_hard(port_type == ConnectorType::OUTPUT);
    return createVariadicOutput(type, label);
}

int VariadicOutputs::getVariadicOutputCount() const
{
    return variadic_outputs_.size();
}

Output *VariadicOutputs::createVariadicOutput(TokenConstPtr type, const std::string& label)
{
    apex_assert_hard(variadic_modifier_);
    auto result = variadic_modifier_->addOutput(type, label.empty() ? std::string("Channel") : label, false);
    if(result) {
        variadic_outputs_.emplace_back(std::dynamic_pointer_cast<Output>(result->shared_from_this()));
        output_count_->set((int) variadic_outputs_.size());
    }
    return result;
}

void VariadicOutputs::removeVariadicOutput(OutputPtr output)
{
    variadic_modifier_->removeOutput(output->getUUID());
    variadic_outputs_.erase(std::find(variadic_outputs_.begin(), variadic_outputs_.end(), output));
}

void VariadicOutputs::removeVariadicOutputById(const UUID& output)
{
    for(auto it = variadic_outputs_.begin(); it != variadic_outputs_.end(); ) {
        OutputPtr o = *it;
        if(o->getUUID() == output) {
            removeVariadicOutput(o);
            return;
        } else {
            ++it;
        }
    }
}

void VariadicOutputs::setupVariadicParameters(Parameterizable &parameters)
{
    output_count_ = csapex::param::ParameterFactory::declareValue("output count", 0);
    parameters.addParameter(output_count_, [this](param::Parameter* p) {
        int count = p->as<int>();
        if(count >= 0) {
            updateOutputs(p->as<int>());
        } else {
            p->set(0);
        }
    });
}

void VariadicOutputs::updateOutputs(int count)
{
    if(count < 0) {
        return;
    }

    apex_assert_hard(variadic_modifier_);

    int current_amount = variadic_outputs_.size();

    if(current_amount > count) {
        bool connected = false;
        for(int i = current_amount; i > count ; i--) {
            OutputPtr out = variadic_outputs_[i - 1];
            if(connected || out->isConnected()) {
                out->disable();
                connected = true;
            } else {
                removeVariadicOutput(out);
            }
        }
    } else {
        int to_add = count - current_amount;
        for(int i = 0 ; i < current_amount; i++) {
            variadic_outputs_.at(i)->enable();
        }
        for(int i = 0 ; i < to_add ; i++) {
            createVariadicOutput(variadic_type_, "Channel");
        }
    }

    portCountChanged();
}


///
/// EVENTS
///


VariadicEvents::VariadicEvents(TokenConstPtr type)
    : VariadicBase(type)
{

}
VariadicEvents::VariadicEvents()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicEvents::createVariadicPort(ConnectorType port_type, TokenConstPtr type, const std::string &label, bool optional)
{
    apex_assert_hard(port_type == ConnectorType::EVENT);
    return createVariadicEvent(label);
}

int VariadicEvents::getVariadicEventCount() const
{
    return variadic_events_.size();
}


Event *VariadicEvents::createVariadicEvent(const std::string& label)
{
    apex_assert_hard(variadic_modifier_);
    auto result = variadic_modifier_->addEvent(label.empty() ? std::string("Event") : label);
    if(result) {
        variadic_events_.emplace_back(std::dynamic_pointer_cast<Event>(result->shared_from_this()));
        event_count_->set((int) variadic_events_.size());
    }
    return result;
}
void VariadicEvents::removeVariadicEvent(EventPtr event)
{
    variadic_events_.erase(std::find(variadic_events_.begin(), variadic_events_.end(), event));
}

void VariadicEvents::removeVariadicEventById(const UUID& event)
{
    variadic_modifier_->removeEvent(event);
    for(auto it = variadic_events_.begin(); it != variadic_events_.end(); ) {
        EventPtr t = *it;
        if(t->getUUID() == event) {
            removeVariadicEvent(t);
            return;
        } else {
            ++it;
        }
    }
}

void VariadicEvents::setupVariadicParameters(Parameterizable &parameters)
{
    event_count_ = csapex::param::ParameterFactory::declareValue("event count", 0);
    parameters.addParameter(event_count_, [this](param::Parameter* p) {
        int count = p->as<int>();
        if(count >= 0) {
            updateEvents(p->as<int>());
        } else {
            p->set(0);
        }
    });
}

void VariadicEvents::updateEvents(int count)
{
    if(count < 0) {
        return;
    }

    apex_assert_hard(variadic_modifier_);

    int current_amount = variadic_events_.size();

    if(current_amount > count) {
        bool connected = false;
        for(int i = current_amount; i > count ; i--) {
            EventPtr t = variadic_events_[i - 1];
            if(connected || t->isConnected()) {
                t->disable();
                connected = true;
            } else {
                removeVariadicEvent(t);
            }
        }
    } else {
        int to_add = count - current_amount;
        for(int i = 0 ; i < current_amount; i++) {
            variadic_events_[i]->enable();
        }
        for(int i = 0 ; i < to_add ; i++) {
            createVariadicEvent("Event");
        }
    }

    portCountChanged();
}



///
/// SLOTS
///


VariadicSlots::VariadicSlots(TokenConstPtr type)
    : VariadicBase(type)
{

}
VariadicSlots::VariadicSlots()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicSlots::createVariadicPort(ConnectorType port_type, TokenConstPtr type, const std::string &label, bool optional)
{
    apex_assert_hard(port_type == ConnectorType::SLOT_T);
    return createVariadicSlot(label, [](){});
}
int VariadicSlots::getVariadicSlotCount() const
{
    return variadic_slots_.size();
}

Slot* VariadicSlots::createVariadicSlot(const std::string& label, std::function<void()> callback)
{
    apex_assert_hard(variadic_modifier_);

    auto result = variadic_modifier_->addSlot(label.empty() ? std::string("Slot") : label, callback);
    if(result) {
        variadic_slots_.emplace_back(std::dynamic_pointer_cast<Slot>(result->shared_from_this()));
        slot_count_->set((int) variadic_slots_.size());
    }
    return result;
}
void VariadicSlots::removeVariadicSlot(SlotPtr slot)
{
    variadic_modifier_->removeSlot(slot->getUUID());
    variadic_slots_.erase(std::find(variadic_slots_.begin(), variadic_slots_.end(), slot));
}

void VariadicSlots::removeVariadicSlotById(const UUID& slot)
{
    for(auto it = variadic_slots_.begin(); it != variadic_slots_.end(); ) {
        SlotPtr s = *it;
        if(s->getUUID() == slot) {
            removeVariadicSlot(s);
            return;
        } else {
            ++it;
        }
    }
}


void VariadicSlots::setupVariadicParameters(Parameterizable &parameters)
{
    slot_count_ = csapex::param::ParameterFactory::declareValue("slot count", 0);
    parameters.addParameter(slot_count_, [this](param::Parameter* p) {
        int count = p->as<int>();
        if(count >= 0) {
            updateSlots(p->as<int>());
        } else {
            p->set(0);
        }
    });
}

void VariadicSlots::updateSlots(int count)
{
    if(count < 0) {
        return;
    }

    apex_assert_hard(variadic_modifier_);

    int current_amount = variadic_slots_.size();

    if(current_amount > count) {
        bool connected = false;
        for(int i = current_amount; i > count ; i--) {
            SlotPtr s = variadic_slots_[i - 1];
            if(connected || s->isConnected()) {
                s->disable();
                connected = true;
            } else {
                removeVariadicSlot(s);
            }
        }
    } else {
        int to_add = count - current_amount;
        for(int i = 0 ; i < current_amount; i++) {
            variadic_slots_[i]->enable();
        }
        for(int i = 0 ; i < to_add ; i++) {
            createVariadicSlot("Slot", [](){});
        }
    }

    portCountChanged();
}





///
/// IO
///


VariadicIO::VariadicIO(TokenConstPtr type)
    : VariadicBase(type), VariadicInputs(type), VariadicOutputs(type)
{

}
VariadicIO::VariadicIO()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicIO::createVariadicPort(ConnectorType port_type, TokenConstPtr type, const std::string& label, bool optional)
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


Variadic::Variadic(TokenConstPtr type)
    : VariadicBase(type), VariadicInputs(type), VariadicOutputs(type), VariadicEvents(type), VariadicSlots(type)
{

}
Variadic::Variadic()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* Variadic::createVariadicPort(ConnectorType port_type, TokenConstPtr type, const std::string& label, bool optional)
{
    apex_assert_hard(variadic_modifier_);
    switch (port_type) {
    case ConnectorType::OUTPUT:
        return createVariadicOutput(type, label);
    case ConnectorType::INPUT:
        return createVariadicInput(type, label, optional);
    case ConnectorType::SLOT_T:
        return createVariadicSlot(label, [](){});
    case ConnectorType::EVENT:
        return createVariadicEvent(label);
    default:
        throw std::logic_error(std::string("Variadic port of type ") + port_type::name(port_type) + " is not supported.");
    }
}

void Variadic::setupVariadicParameters(Parameterizable &parameters)
{
    VariadicInputs::setupVariadicParameters(parameters);
    VariadicOutputs::setupVariadicParameters(parameters);
    VariadicEvents::setupVariadicParameters(parameters);
    VariadicSlots::setupVariadicParameters(parameters);
}
