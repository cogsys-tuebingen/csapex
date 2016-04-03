#ifndef VARIADIC_IO_H
#define VARIADIC_IO_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/model/parameterizable.h>

namespace csapex
{

class VariadicBase
{
public:
    ~VariadicBase();

    virtual Connectable* createVariadicPort(bool output, ConnectionTypeConstPtr type, const std::string& label, bool optional) = 0;

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
    virtual Connectable* createVariadicPort(bool output, ConnectionTypeConstPtr type, const std::string& label, bool optional) override;

protected:
    VariadicInputs(ConnectionTypeConstPtr type);
    VariadicInputs();

    virtual void setupVariadicParameters(Parameterizable &parameters) override;

private:
    void updateInputs(int input_count);

private:
    param::ParameterPtr input_count_;
};



class VariadicOutputs : public virtual VariadicBase
{
public:
    virtual Connectable* createVariadicPort(bool output, ConnectionTypeConstPtr type, const std::string& label, bool optional) override;

protected:
    VariadicOutputs(ConnectionTypeConstPtr type);
    VariadicOutputs();

    virtual void setupVariadicParameters(Parameterizable &parameters) override;

private:
    void updateOutputs(int output_count);

private:
    param::ParameterPtr output_count_;
};



class VariadicIO : public VariadicInputs, public VariadicOutputs
{
public:
    virtual Connectable* createVariadicPort(bool output, ConnectionTypeConstPtr type, const std::string& label, bool optional) override;

protected:
    VariadicIO(ConnectionTypeConstPtr type);
    VariadicIO();

    virtual void setupVariadicParameters(Parameterizable &parameters) final override;
};
}

#endif // VARIADIC_IO_H
