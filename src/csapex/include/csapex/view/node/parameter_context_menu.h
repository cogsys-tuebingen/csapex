#ifndef PARAMETER_CONTEXT_MENU_H
#define PARAMETER_CONTEXT_MENU_H

/// COMPONENT
#include <csapex/utility/context_menu_handler.h>

/// PROJECT
#include <utils_param/param_fwd.h>

namespace csapex
{

class ParameterContextMenu : public ContextMenuHandler
{
public:
    ParameterContextMenu(param::ParameterWeakPtr p);

    void doShowContextMenu(const QPoint& pt);

private:
    param::ParameterWeakPtr param_;
};
}

#endif // PARAMETER_CONTEXT_MENU_H
