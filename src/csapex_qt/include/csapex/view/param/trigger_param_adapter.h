#ifndef TRIGGER_PARAM_ADAPTER_H
#define TRIGGER_PARAM_ADAPTER_H

/// COMPONENT
#include <csapex/view/param/param_adapter.h>
#include <csapex/param/trigger_parameter.h>

class QHBoxLayout;

namespace csapex
{
class ParameterContextMenu;

class CSAPEX_QT_EXPORT TriggerParameterAdapter : public ParameterAdapter
{
public:
    TriggerParameterAdapter(param::TriggerParameter::Ptr p);

    virtual QWidget* setup(QBoxLayout* layout, const std::string& display_name) override;
    virtual void setupContextMenu(ParameterContextMenu* context_handler) override;

private:
    param::TriggerParameterPtr value_p_;
};

}  // namespace csapex

#endif  // TRIGGER_PARAM_ADAPTER_H
