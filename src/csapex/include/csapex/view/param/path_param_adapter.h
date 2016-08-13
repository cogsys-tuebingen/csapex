#ifndef PATH_PARAM_ADAPTER_H
#define PATH_PARAM_ADAPTER_H

/// COMPONENT
#include <csapex/view/param/param_adapter.h>
#include <csapex/param/path_parameter.h>

class QHBoxLayout;

namespace csapex
{
class ParameterContextMenu;

class CSAPEX_QT_EXPORT PathParameterAdapter : public ParameterAdapter
{
public:
    PathParameterAdapter(param::PathParameter::Ptr p);

    virtual QWidget* setup(QBoxLayout* layout, const std::string& display_name) override;

private:
    param::PathParameterPtr path_p_;
};


}

#endif // PATH_PARAM_ADAPTER_H
