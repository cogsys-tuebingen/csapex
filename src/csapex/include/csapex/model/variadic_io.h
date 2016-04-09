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

class VariadicBase
{
public:
    ~VariadicBase();

    virtual Connectable* createVariadicPort(ConnectorType port_type, ConnectionTypeConstPtr type, const std::string& label, bool optional) = 0;

protected:
    VariadicBase(ConnectionTypeConstPtr type);
    VariadicBase();

    void setupVariadic(csapex::NodeModifier& node_modifier);
    virtual void setupVariadicParameters(Parameterizable &parameters) = 0;

    virtual void portCountChanged();

protected:
    ConnectionTypeConstPtr variadic_type_;
    csapex::NodeModifier* variadic_modifier_;
};


class VariadicInputs : public virtual VariadicBase
{
public:
    virtual Connectable* createVariadicInput(ConnectionTypeConstPtr type, const std::string& label, bool optional);
    virtual Connectable* createVariadicPort(ConnectorType port_type, ConnectionTypeConstPtr type, const std::string& label, bool optional) override;

protected:
    VariadicInputs(ConnectionTypeConstPtr type);
    VariadicInputs();

    virtual void setupVariadicParameters(Parameterizable &parameters) override;

private:
    void updateInputs(int input_count);

private:
    param::ParameterPtr input_count_;
    std::vector<Input*> variadic_inputs_;
};



class VariadicOutputs : public virtual VariadicBase
{
public:
    virtual Connectable* createVariadicOutput(ConnectionTypeConstPtr type, const std::string& label);
    virtual Connectable* createVariadicPort(ConnectorType port_type, ConnectionTypeConstPtr type, const std::string& label, bool optional) override;

protected:
    VariadicOutputs(ConnectionTypeConstPtr type);
    VariadicOutputs();

    virtual void setupVariadicParameters(Parameterizable &parameters) override;

private:
    void updateOutputs(int output_count);

private:
    param::ParameterPtr output_count_;
    std::vector<Output*> variadic_outputs_;
};




class VariadicTriggers : public virtual VariadicBase
{
public:
    virtual Connectable* createVariadicTrigger(const std::string& label);
    virtual Connectable* createVariadicPort(ConnectorType port_type, ConnectionTypeConstPtr type, const std::string& label, bool optional) override;

protected:
    VariadicTriggers(ConnectionTypeConstPtr type);
    VariadicTriggers();

    virtual void setupVariadicParameters(Parameterizable &parameters) override;

private:
    void updateTriggers(int trigger_count);

private:
    param::ParameterPtr trigger_count_;
    std::vector<Trigger*> variadic_triggers_;
};



class VariadicSlots: public virtual VariadicBase
{
public:
    virtual Connectable* createVariadicSlot(const std::string& label);
    virtual Connectable* createVariadicPort(ConnectorType port_type, ConnectionTypeConstPtr type, const std::string& label, bool optional) override;

protected:
    VariadicSlots(ConnectionTypeConstPtr type);
    VariadicSlots();

    virtual void setupVariadicParameters(Parameterizable &parameters) override;

private:
    void updateSlots(int slot_count);

private:
    param::ParameterPtr slot_count_;
    std::vector<Slot*> variadic_slots_;
};



class VariadicIO : public VariadicInputs, public VariadicOutputs
{
public:
    virtual Connectable* createVariadicPort(ConnectorType port_type, ConnectionTypeConstPtr type, const std::string& label, bool optional) override;

protected:
    VariadicIO(ConnectionTypeConstPtr type);
    VariadicIO();

    virtual void setupVariadicParameters(Parameterizable &parameters) final override;
};


class Variadic : public VariadicInputs, public VariadicOutputs, public VariadicTriggers, public VariadicSlots
{
public:
    virtual Connectable* createVariadicPort(ConnectorType port_type, ConnectionTypeConstPtr type, const std::string& label, bool optional) override;

protected:
    Variadic(ConnectionTypeConstPtr type);
    Variadic();

    virtual void setupVariadicParameters(Parameterizable &parameters) final override;
};
}

#endif // VARIADIC_IO_H
