#ifndef PARAMETER_CONTEXT_MENU_H
#define PARAMETER_CONTEXT_MENU_H

/// COMPONENT
#include <csapex/view/utility/context_menu_handler.h>

/// PROJECT
#include <csapex/param/param_fwd.h>

namespace csapex
{

class ParameterContextMenu : public ContextMenuHandler
{
public:
    ParameterContextMenu(csapex::param::ParameterWeakPtr p);

    void doShowContextMenu(const QPoint& pt);

private:
    csapex::param::ParameterWeakPtr param_;
};
}

#endif // PARAMETER_CONTEXT_MENU_H
