#ifndef OUTPUT_PROGRESS_PARAM_ADAPTER_H
#define OUTPUT_PROGRESS_PARAM_ADAPTER_H

/// COMPONENT
#include <csapex/view/param/param_adapter.h>
#include <csapex/param/output_progress_parameter.h>

class QHBoxLayout;

namespace csapex
{
class ParameterContextMenu;

class CSAPEX_QT_EXPORT OutputProgressParameterAdapter : public ParameterAdapter
{
public:
    OutputProgressParameterAdapter(param::OutputProgressParameter::Ptr p);

    virtual QWidget* setup(QBoxLayout* layout, const std::string& display_name) override;

private:
    param::OutputProgressParameterPtr op_p_;
};


}

#endif // OUTPUT_PROGRESS_PARAM_ADAPTER_H
