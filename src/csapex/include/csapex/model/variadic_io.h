#ifndef VARIADIC_IO_H
#define VARIADIC_IO_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/model/parameterizable.h>

namespace csapex
{

class VariadicInputs
{
public:
    ~VariadicInputs();

    virtual Connectable* createVariadicPort(bool output, ConnectionTypeConstPtr type, const std::string& label, bool optional);

protected:
    VariadicInputs(ConnectionTypeConstPtr type);
    VariadicInputs();

    void setup(csapex::NodeModifier& node_modifier);
    void setupParameters(Parameterizable &parameters);

private:
    void updateInputs(int input_count);

private:
    ConnectionTypeConstPtr type_;
    csapex::NodeModifier* modifier_;

    param::ParameterPtr count_p_;
};

}

#endif // VARIADIC_IO_H
