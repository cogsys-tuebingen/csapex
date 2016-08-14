#ifndef VALUE_PARAM_ADAPTER_H
#define VALUE_PARAM_ADAPTER_H

/// COMPONENT
#include <csapex/view/param/param_adapter.h>
#include <csapex/param/value_parameter.h>

class QHBoxLayout;

namespace csapex
{
class ParameterContextMenu;

class CSAPEX_QT_EXPORT ValueParameterAdapter : public ParameterAdapter
{
public:
    ValueParameterAdapter(param::ValueParameter::Ptr p);

    virtual QWidget* setup(QBoxLayout* layout, const std::string& display_name) override;
    virtual void setupContextMenu(ParameterContextMenu *context_handler) override;

private:
    param::ValueParameterPtr value_p_;

private:
    static const int DEFAULT_INT_STEP_SIZE;
    static const double DEFAULT_DOUBLE_STEP_SIZE;
};


}

#endif // VALUE_PARAM_ADAPTER_H
