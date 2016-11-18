/// HEADER
#include <csapex/view/node/parameter_context_menu.h>

/// PROJECT
#include <csapex/param/parameter.h>

using namespace csapex;

ParameterContextMenu::ParameterContextMenu(csapex::param::ParameterWeakPtr p)
    : param_(p)
{

}

void ParameterContextMenu::addAction(QAction *action, const std::function<void()>& callback)
{
    actions_[action] = callback;
}

void ParameterContextMenu::doShowContextMenu(const QPoint& global_pt)
{
    auto param = param_.lock();
    if(!param) {
        return;
    }
    QMenu menu;
    ContextMenuHandler::addHeader(menu, std::string("Parameter: ") + param->name());

    if(!actions_.empty()) {
        for(auto pair : actions_) {
            menu.addAction(pair.first);
        }

        menu.addSeparator();
    }

    QAction* connectable = new QAction("make connectable", &menu);
    connectable->setCheckable(true);
    connectable->setChecked(param->isInteractive());
    connectable->setIcon(QIcon(":/connector.png"));

    connectable->setIconVisibleInMenu(true);
    menu.addAction(connectable);

    QAction* selectedItem = menu.exec(global_pt);
    if (selectedItem) {
        if(selectedItem == connectable) {
            param->setInteractive(!param->isInteractive());
        } else {
            actions_[selectedItem]();
        }
    }
}
