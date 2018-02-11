#ifndef OUTPUT_TEXT_ADAPTER_H
#define OUTPUT_TEXT_ADAPTER_H

/// COMPONENT
#include <csapex/view/param/param_adapter.h>
#include <csapex/param/output_text_parameter.h>

class QHBoxLayout;

namespace csapex
{
class ParameterContextMenu;

class CSAPEX_QT_EXPORT OutputTextParameterAdapter : public ParameterAdapter
{
public:
    OutputTextParameterAdapter(param::OutputTextParameter::Ptr p);

    virtual QWidget* setup(QBoxLayout* layout, const std::string& display_name) override;

private:
    param::OutputTextParameterPtr op_p_;
};


}
#endif // OUTPUT_TEXT_ADAPTER_H
