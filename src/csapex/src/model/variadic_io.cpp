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
#include <csapex/param/string_list_parameter.h>

using namespace csapex;

VariadicBase::VariadicBase(TokenDataConstPtr type)
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

VariadicInputs::VariadicInputs(TokenDataConstPtr type)
    : VariadicBase(type)
{

}
VariadicInputs::VariadicInputs()
    : VariadicInputs(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicInputs::createVariadicPort(ConnectorType port_type, TokenDataConstPtr type, const std::string &label, bool optional)
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

Input *VariadicInputs::createVariadicInput(TokenDataConstPtr type, const std::string& label, bool optional)
{
    apex_assert_hard(variadic_modifier_);
    Input* result = variadic_modifier_->addInput(type, label.empty() ? std::string("Input") : label, optional);
    if(result) {
        variadic_inputs_.push_back(std::dynamic_pointer_cast<Input>(result->shared_from_this()));
        input_count_->set((int) variadic_inputs_.size());

        if(variadic_inputs_.size() >= input_names_->count()) {
            input_names_->add(label);
        }
        int index = variadic_inputs_.size() - 1;
        result->labelChanged.connect([this, index](const std::string& label){
            input_names_->setAt(index, label);
        });
    }
    return result;
}

InputPtr VariadicInputs::getVariadicInput(std::size_t index)
{
    return variadic_inputs_.at(index);
}


void VariadicInputs::setupVariadicParameters(Parameterizable &parameters)
{
    input_count_ = csapex::param::ParameterFactory::declareValue("input count", 0);
    parameters.addHiddenParameter(input_count_, [this](param::Parameter* p) {
        int count = p->as<int>();
        if(count >= 0) {
            updateInputs(p->as<int>());
        } else {
            p->set(0);
        }
    });

    input_names_ = std::make_shared<csapex::param::StringListParameter>("input names", param::ParameterDescription("variadic input names"));
    parameters.addHiddenParameter(input_names_);
}


void VariadicInputs::updateInputs(int count)
{
    if(count < 0) {
        return;
    }

    apex_assert_hard(variadic_modifier_);

    int current_amount = variadic_inputs_.size();

    std::vector<std::string> variadic_names = input_names_->getValues();

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
            std::string label;
            if(variadic_inputs_.size() < variadic_names.size()) {
                label = variadic_names.at(variadic_inputs_.size());
            } else {
                label = "Input";
            }
            createVariadicInput(variadic_type_, label, true);
        }
    }

    portCountChanged();
}


///
/// OUTPUTS
///


VariadicOutputs::VariadicOutputs(TokenDataConstPtr type)
    : VariadicBase(type)
{

}
VariadicOutputs::VariadicOutputs()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicOutputs::createVariadicPort(ConnectorType port_type, TokenDataConstPtr type, const std::string &label, bool optional)
{
    apex_assert_hard(port_type == ConnectorType::OUTPUT);
    return createVariadicOutput(type, label);
}

int VariadicOutputs::getVariadicOutputCount() const
{
    return variadic_outputs_.size();
}

Output *VariadicOutputs::createVariadicOutput(TokenDataConstPtr type, const std::string& label)
{
    apex_assert_hard(variadic_modifier_);
    auto result = variadic_modifier_->addOutput(type, label.empty() ? std::string("Output") : label);
    if(result) {
        variadic_outputs_.emplace_back(std::dynamic_pointer_cast<Output>(result->shared_from_this()));
        output_count_->set((int) variadic_outputs_.size());

        if(variadic_outputs_.size() >= output_names_->count()) {
            output_names_->add(label);
        }
        int index = variadic_outputs_.size() - 1;
        result->labelChanged.connect([this, index](const std::string& label){
            output_names_->setAt(index, label);
        });
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
    parameters.addHiddenParameter(output_count_, [this](param::Parameter* p) {
        int count = p->as<int>();
        if(count >= 0) {
            updateOutputs(p->as<int>());
        } else {
            p->set(0);
        }
    });

    output_names_ = std::make_shared<csapex::param::StringListParameter>("output names", param::ParameterDescription("variadic output names"));
    parameters.addHiddenParameter(output_names_);
}

OutputPtr VariadicOutputs::getVariadicOutput(std::size_t index)
{
    return variadic_outputs_.at(index);
}


void VariadicOutputs::updateOutputs(int count)
{
    if(count < 0) {
        return;
    }

    apex_assert_hard(variadic_modifier_);

    int current_amount = variadic_outputs_.size();

    std::vector<std::string> variadic_names = output_names_->getValues();

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
            std::string label;
            if(variadic_outputs_.size() < variadic_names.size()) {
                label = variadic_names.at(variadic_outputs_.size());
            } else {
                label = "Output";
            }
            createVariadicOutput(variadic_type_, label);
        }
    }

    portCountChanged();
}


///
/// EVENTS
///


VariadicEvents::VariadicEvents(TokenDataConstPtr type)
    : VariadicBase(type)
{

}
VariadicEvents::VariadicEvents()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicEvents::createVariadicPort(ConnectorType port_type, TokenDataConstPtr type, const std::string &label, bool optional)
{
    apex_assert_hard(port_type == ConnectorType::EVENT);
    return createVariadicEvent(type, label);
}

int VariadicEvents::getVariadicEventCount() const
{
    return variadic_events_.size();
}


Event *VariadicEvents::createVariadicEvent(TokenDataConstPtr type, const std::string& label)
{
    apex_assert_hard(variadic_modifier_);
    auto result = variadic_modifier_->addEvent(type, label.empty() ? std::string("Event") : label);
    if(result) {
        variadic_events_.emplace_back(std::dynamic_pointer_cast<Event>(result->shared_from_this()));
        event_count_->set((int) variadic_events_.size());

        if(variadic_events_.size() >= event_names_->count()) {
            event_names_->add(label);
        }
        int index = variadic_events_.size() - 1;
        result->labelChanged.connect([this, index](const std::string& label){
            event_names_->setAt(index, label);
        });
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
    parameters.addHiddenParameter(event_count_, [this](param::Parameter* p) {
        int count = p->as<int>();
        if(count >= 0) {
            updateEvents(p->as<int>());
        } else {
            p->set(0);
        }
    });

    event_names_ = std::make_shared<csapex::param::StringListParameter>("event names", param::ParameterDescription("variadic event names"));
    parameters.addHiddenParameter(event_names_);
}

EventPtr VariadicEvents::getVariadicEvent(std::size_t index)
{
    return variadic_events_.at(index);
}

void VariadicEvents::updateEvents(int count)
{
    if(count < 0) {
        return;
    }

    apex_assert_hard(variadic_modifier_);

    int current_amount = variadic_events_.size();

    std::vector<std::string> variadic_names = event_names_->getValues();

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
            std::string label;
            if(variadic_events_.size() < variadic_names.size()) {
                label = variadic_names.at(variadic_events_.size());
            } else {
                label = "Event";
            }
            createVariadicEvent(connection_types::makeEmpty<connection_types::AnyMessage>(), label);
        }
    }

    portCountChanged();
}



///
/// SLOTS
///


VariadicSlots::VariadicSlots(TokenDataConstPtr type)
    : VariadicBase(type)
{

}
VariadicSlots::VariadicSlots()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicSlots::createVariadicPort(ConnectorType port_type, TokenDataConstPtr type, const std::string &label, bool optional)
{
    apex_assert_hard(port_type == ConnectorType::SLOT_T);
    return createVariadicSlot(type, label, [](const TokenPtr&){});
}
int VariadicSlots::getVariadicSlotCount() const
{
    return variadic_slots_.size();
}

Slot* VariadicSlots::createVariadicSlot(TokenDataConstPtr type, const std::string& label, std::function<void (const TokenPtr &)> callback, bool active, bool asynchronous)
{
    apex_assert_hard(variadic_modifier_);

    auto result = variadic_modifier_->addSlot(type, label.empty() ? std::string("Slot") : label, callback, active, asynchronous);
    registerSlot(result);
    return result;
}

Slot* VariadicSlots::createVariadicSlot(TokenDataConstPtr type, const std::string& label, std::function<void (Slot* slot, const TokenPtr &)> callback, bool active, bool asynchronous)
{
    apex_assert_hard(variadic_modifier_);

    auto result = variadic_modifier_->addSlot(type, label.empty() ? std::string("Slot") : label, callback, active, asynchronous);
    registerSlot(result);
    return result;
}

void VariadicSlots::registerSlot(Slot* slot)
{
    if(slot) {
        variadic_slots_.emplace_back(std::dynamic_pointer_cast<Slot>(slot->shared_from_this()));
        slot_count_->set((int) variadic_slots_.size());

        if(variadic_slots_.size() >= slot_names_->count()) {
            slot_names_->add(slot->getLabel());
        }

        int index = variadic_slots_.size() - 1;
        slot->labelChanged.connect([this, index](const std::string& label){
            slot_names_->setAt(index, label);
        });
    }
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


SlotPtr VariadicSlots::getVariadicSlot(std::size_t index)
{
    return variadic_slots_.at(index);
}


void VariadicSlots::setupVariadicParameters(Parameterizable &parameters)
{
    slot_count_ = csapex::param::ParameterFactory::declareValue("slot count", 0);
    parameters.addHiddenParameter(slot_count_, [this](param::Parameter* p) {
        int count = p->as<int>();
        if(count >= 0) {
            updateSlots(p->as<int>());
        } else {
            p->set(0);
        }
    });

    slot_names_ = std::make_shared<csapex::param::StringListParameter>("slot names", param::ParameterDescription("variadic slot names"));
    parameters.addHiddenParameter(slot_names_);
}

void VariadicSlots::updateSlots(int count)
{
    if(count < 0) {
        return;
    }

    apex_assert_hard(variadic_modifier_);

    int current_amount = variadic_slots_.size();

    std::vector<std::string> variadic_names = slot_names_->getValues();

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
            std::string label;
            if(variadic_slots_.size() < variadic_names.size()) {
                label = variadic_names.at(variadic_slots_.size());
            } else {
                label = "Slot";
            }
            createVariadicSlot(connection_types::makeEmpty<connection_types::AnyMessage>(), label, [](const TokenPtr&){});
        }
    }

    portCountChanged();
}





///
/// IO
///


VariadicIO::VariadicIO(TokenDataConstPtr type)
    : VariadicBase(type), VariadicInputs(type), VariadicOutputs(type)
{

}
VariadicIO::VariadicIO()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* VariadicIO::createVariadicPort(ConnectorType port_type, TokenDataConstPtr type, const std::string& label, bool optional)
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


Variadic::Variadic(TokenDataConstPtr type)
    : VariadicBase(type), VariadicInputs(type), VariadicOutputs(type), VariadicEvents(type), VariadicSlots(type)
{

}
Variadic::Variadic()
    : VariadicBase(connection_types::makeEmpty<connection_types::AnyMessage>())
{
}

Connectable* Variadic::createVariadicPort(ConnectorType port_type, TokenDataConstPtr type, const std::string& label, bool optional)
{
    apex_assert_hard(variadic_modifier_);
    switch (port_type) {
    case ConnectorType::OUTPUT:
        return createVariadicOutput(type, label);
    case ConnectorType::INPUT:
        return createVariadicInput(type, label, optional);
    case ConnectorType::SLOT_T:
        return createVariadicSlot(type, label, [](const TokenPtr&){});
    case ConnectorType::EVENT:
        return createVariadicEvent(type, label);
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
