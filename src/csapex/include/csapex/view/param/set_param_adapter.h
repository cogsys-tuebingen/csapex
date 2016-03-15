#ifndef SET_PARAM_ADAPTER_H
#define SET_PARAM_ADAPTER_H

/// COMPONENT
#include <csapex/view/param/param_adapter.h>
#include <csapex/param/set_parameter.h>

/// SYSTEM
#include <QPointer>

class QHBoxLayout;
class QComboBox;

namespace csapex
{
class ParameterContextMenu;

class SetParameterAdapter : public ParameterAdapter
{
public:
    SetParameterAdapter(param::SetParameter::Ptr p);

    virtual void setup(QBoxLayout* layout, const std::string& display_name) override;

private:
    void updateSetParameterScope(QPointer<QComboBox> combo);

private:
    param::SetParameterPtr set_p_;
};


}

#endif // SET_PARAM_ADAPTER_H
