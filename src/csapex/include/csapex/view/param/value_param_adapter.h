#ifndef VALUE_PARAM_ADAPTER_H
#define VALUE_PARAM_ADAPTER_H

/// COMPONENT
#include <csapex/view/param/param_adapter.h>
#include <csapex/param/value_parameter.h>

class QHBoxLayout;

namespace csapex
{
class ParameterContextMenu;

class ValueParameterAdapter : public ParameterAdapter
{
public:
    ValueParameterAdapter(param::ValueParameter::Ptr p);

    virtual void setup(QBoxLayout* layout, const std::string& display_name) override;

private:
    param::ValueParameterPtr value_p_;
};


}

#endif // VALUE_PARAM_ADAPTER_H
