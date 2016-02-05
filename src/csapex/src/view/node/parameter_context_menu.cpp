/// HEADER
#include <csapex/view/node/parameter_context_menu.h>

/// COMPONENT
#include <csapex/view/designer/graph_view.h>

/// PROJECT
#include <csapex/param/parameter.h>

/// SYSTEM
#include <QApplication>

using namespace csapex;

ParameterContextMenu::ParameterContextMenu(csapex::param::ParameterWeakPtr p)
    : param_(p)
{

}

void ParameterContextMenu::addAction(QAction *action, const std::function<void()>& callback)
{
    actions_[action] = callback;
}

void ParameterContextMenu::doShowContextMenu(const QPoint& pt)
{
    auto param = param_.lock();
    if(!param) {
        return;
    }

    QWidget* w = dynamic_cast<QWidget*>(parent());
    if(!w) {
        return;
    }

    GraphView* view = QApplication::activeWindow()->findChild<GraphView*>();

    QWidget* real_parent = w;
    while(real_parent->parentWidget()) {
        real_parent = real_parent->parentWidget();
    }

    QPoint gpt = view->mapToGlobal(view->mapFromScene(pt));

    QMenu menu;
    ContextMenuHandler::addHeader(menu, std::string("Parameter: ") + param->name());

    if(!actions_.empty()) {
        for(auto pair : actions_) {
            menu.addAction(pair.first);
        }

        menu.addSeparator();
    }

    QAction* connectable = new QAction("connectable", &menu);
    connectable->setCheckable(true);
    connectable->setChecked(param->isInteractive());
    connectable->setIcon(QIcon(":/connector.png"));

    connectable->setIconVisibleInMenu(true);
    menu.addAction(connectable);

    QAction* selectedItem = menu.exec(gpt);
    if (selectedItem) {
        if(selectedItem == connectable) {
            param->setInteractive(!param->isInteractive());
        } else {
            actions_[selectedItem]();
        }
    }
}
