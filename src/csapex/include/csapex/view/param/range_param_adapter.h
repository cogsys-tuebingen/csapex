#ifndef RANGE_PARAM_ADAPTER_H
#define RANGE_PARAM_ADAPTER_H

/// COMPONENT
#include <csapex/view/param/param_adapter.h>
#include <csapex/param/range_parameter.h>

namespace csapex
{

class RangeParameterAdapter : public ParameterAdapter
{
public:
    RangeParameterAdapter(param::Proxy<param::RangeParameter>::Ptr p);

    virtual void setup(QBoxLayout* layout, const std::string& display_name) override;

private:
    param::RangeParameterPtr range_p_;
};


}

#endif // RANGE_PARAM_ADAPTER_H
