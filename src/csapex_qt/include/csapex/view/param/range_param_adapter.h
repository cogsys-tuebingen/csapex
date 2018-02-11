#ifndef RANGE_PARAM_ADAPTER_H
#define RANGE_PARAM_ADAPTER_H

/// COMPONENT
#include <csapex/view/param/param_adapter.h>
#include <csapex/param/range_parameter.h>

namespace csapex
{

class CSAPEX_QT_EXPORT RangeParameterAdapter : public ParameterAdapter
{
public:
    RangeParameterAdapter(param::RangeParameter::Ptr p);

    virtual QWidget* setup(QBoxLayout* layout, const std::string& display_name) override;

    virtual void setupContextMenu(ParameterContextMenu* context_handler) override;

private:
    template <typename T, typename Slider, typename Spinbox>
    void genericSetup();

private:
    param::RangeParameterPtr range_p_;

    QHBoxLayout* internal_layout;
};


}

#endif // RANGE_PARAM_ADAPTER_H
