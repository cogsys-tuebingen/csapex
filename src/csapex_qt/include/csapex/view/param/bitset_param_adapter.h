#ifndef BITSET_PARAM_ADAPTER_H
#define BITSET_PARAM_ADAPTER_H

/// COMPONENT
#include <csapex/view/param/param_adapter.h>
#include <csapex/param/bitset_parameter.h>

/// SYSTEM
#include <QPointer>

class QHBoxLayout;
class QGroupBox;

namespace csapex
{
class ParameterContextMenu;

class CSAPEX_QT_EXPORT BitSetParameterAdapter : public ParameterAdapter
{
public:
    BitSetParameterAdapter(param::BitSetParameter::Ptr p);

    QWidget* setup(QBoxLayout* layout, const std::string& display_name) override;
    void setupContextMenu(ParameterContextMenu* context_handler) override;

private:
    void setupAgain();

private:
    param::BitSetParameterPtr bitset_p_;
    QPointer<QGroupBox> group;
};

}  // namespace csapex

#endif  // BITSET_PARAM_ADAPTER_H
