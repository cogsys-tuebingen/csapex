#ifndef BITSET_PARAM_ADAPTER_H
#define BITSET_PARAM_ADAPTER_H

/// COMPONENT
#include <csapex/view/param/param_adapter.h>
#include <csapex/param/bitset_parameter.h>

class QHBoxLayout;

namespace csapex
{
class ParameterContextMenu;

class BitSetParameterAdapter : public ParameterAdapter
{
public:
    BitSetParameterAdapter(param::BitSetParameter::Ptr p);

    virtual void setup(QBoxLayout* layout, const std::string& display_name) override;

private:
    param::BitSetParameterPtr bitset_p_;
};


}

#endif // BITSET_PARAM_ADAPTER_H
