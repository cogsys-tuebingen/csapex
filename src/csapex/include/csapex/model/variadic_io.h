#ifndef VARIADIC_IO_H
#define VARIADIC_IO_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/model/parameterizable.h>
#include <csapex/model/connector_type.h>

namespace csapex
{

class CSAPEX_EXPORT VariadicBase
{
public:
    ~VariadicBase();

    virtual Connectable* createVariadicPort(ConnectorType port_type, TokenDataConstPtr type, const std::string& label, bool optional) = 0;

protected:
    VariadicBase(TokenDataConstPtr type);
    VariadicBase();

    void setupVariadic(csapex::NodeModifier& node_modifier);
    virtual void setupVariadicParameters(Parameterizable &parameters) = 0;

    virtual void portCountChanged();

protected:
    TokenDataConstPtr variadic_type_;
    csapex::NodeModifier* variadic_modifier_;
};


class CSAPEX_EXPORT VariadicInputs : public virtual VariadicBase
{
public:
    virtual Input* createVariadicInput(TokenDataConstPtr type, const std::string& label, bool optional);
    virtual void removeVariadicInput(InputPtr input);
    void removeVariadicInputById(const UUID& input);
    virtual Connectable* createVariadicPort(ConnectorType port_type, TokenDataConstPtr type, const std::string& label, bool optional) override;

    int getVariadicInputCount() const;

    InputPtr getVariadicInput(std::size_t index);

protected:
    VariadicInputs(TokenDataConstPtr type);
    VariadicInputs();

    virtual void setupVariadicParameters(Parameterizable &parameters) override;

private:
    void updateInputs(int input_count);

private:
    param::ParameterPtr input_count_;
    param::StringListParameterPtr input_names_;

protected:
    std::vector<InputPtr> variadic_inputs_;
};



class CSAPEX_EXPORT VariadicOutputs : public virtual VariadicBase
{
public:
    virtual Output* createVariadicOutput(TokenDataConstPtr type, const std::string& label);
    virtual void removeVariadicOutput(OutputPtr output);
    void removeVariadicOutputById(const UUID& output);
    virtual Connectable* createVariadicPort(ConnectorType port_type, TokenDataConstPtr type, const std::string& label, bool optional) override;

    int getVariadicOutputCount() const;

protected:
    VariadicOutputs(TokenDataConstPtr type);
    VariadicOutputs();

    virtual void setupVariadicParameters(Parameterizable &parameters) override;

    OutputPtr getVariadicOutput(std::size_t index);

private:
    void updateOutputs(int output_count);

private:
    param::ParameterPtr output_count_;
    param::StringListParameterPtr output_names_;

protected:
    std::vector<OutputPtr> variadic_outputs_;
};




class CSAPEX_EXPORT VariadicEvents : public virtual VariadicBase
{
public:
    virtual Event* createVariadicEvent(TokenDataConstPtr type, const std::string& label);
    virtual void removeVariadicEvent(EventPtr trigger);
    void removeVariadicEventById(const UUID& trigger);
    virtual Connectable* createVariadicPort(ConnectorType port_type, TokenDataConstPtr type, const std::string& label, bool optional) override;

    int getVariadicEventCount() const;

protected:
    VariadicEvents(TokenDataConstPtr type);
    VariadicEvents();

    virtual void setupVariadicParameters(Parameterizable &parameters) override;

    EventPtr getVariadicEvent(std::size_t index);

private:
    void updateEvents(int trigger_count);

private:
    param::ParameterPtr event_count_;
    param::StringListParameterPtr event_names_;

protected:
    std::vector<EventPtr> variadic_events_;
};



class CSAPEX_EXPORT VariadicSlots: public virtual VariadicBase
{
public:
    virtual Slot* createVariadicSlot(TokenDataConstPtr type, const std::string& label, std::function<void (Slot* slot, const TokenPtr&)> callback, bool active = false, bool asynchronous = false);
    virtual Slot* createVariadicSlot(TokenDataConstPtr type, const std::string& label, std::function<void (const TokenPtr&)> callback, bool active = false, bool asynchronous = false);
    virtual void removeVariadicSlot(SlotPtr slot);
    void removeVariadicSlotById(const UUID& slot);
    virtual Connectable* createVariadicPort(ConnectorType port_type, TokenDataConstPtr type, const std::string& label, bool optional) override;

    int getVariadicSlotCount() const;

    SlotPtr getVariadicSlot(std::size_t index);

protected:
    VariadicSlots(TokenDataConstPtr type);
    VariadicSlots();

    virtual void setupVariadicParameters(Parameterizable &parameters) override;

private:
    void updateSlots(int slot_count);
    void registerSlot(Slot *result);

private:
    param::ParameterPtr slot_count_;
    param::StringListParameterPtr slot_names_;


protected:
    std::vector<SlotPtr> variadic_slots_;
};



class CSAPEX_EXPORT VariadicIO : public VariadicInputs, public VariadicOutputs
{
public:
    virtual Connectable* createVariadicPort(ConnectorType port_type, TokenDataConstPtr type, const std::string& label, bool optional) override;

protected:
    VariadicIO(TokenDataConstPtr type);
    VariadicIO();

    virtual void setupVariadicParameters(Parameterizable &parameters) final override;
};


class CSAPEX_EXPORT Variadic : public VariadicInputs, public VariadicOutputs, public VariadicEvents, public VariadicSlots
{
public:
    virtual Connectable* createVariadicPort(ConnectorType port_type, TokenDataConstPtr type, const std::string& label, bool optional) override;

protected:
    Variadic(TokenDataConstPtr type);
    Variadic();

    virtual void setupVariadicParameters(Parameterizable &parameters) final override;
};
}

#endif // VARIADIC_IO_H
