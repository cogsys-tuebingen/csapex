#ifndef PARAMETER_CONTEXT_MENU_H
#define PARAMETER_CONTEXT_MENU_H

/// COMPONENT
#include <csapex_qt/export.h>
#include <csapex/view/utility/context_menu_handler.h>

/// PROJECT
#include <csapex/param/param_fwd.h>
#include <csapex/utility/delegate.h>

/// SYSTEM
#include <functional>

namespace csapex
{
class CSAPEX_QT_EXPORT ParameterContextMenu : public ContextMenuHandler
{
public:
    ParameterContextMenu(csapex::param::ParameterWeakPtr p);

    void doShowContextMenu(const QPoint& pt) override;

    void addAction(QAction* action, const std::function<void()>& callback);

private:
    csapex::param::ParameterWeakPtr param_;

    std::map<QAction*, std::function<void()>> actions_;
};
}  // namespace csapex

#endif  // PARAMETER_CONTEXT_MENU_H
