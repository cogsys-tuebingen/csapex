/// HEADER
#include <csapex/view/utility/context_menu_handler.h>

/// SYSTEM
#include <QAction>
#include <QGraphicsProxyWidget>
#include <QGraphicsScene>
#include <QGraphicsView>

using namespace csapex;

ContextMenuHandler::~ContextMenuHandler()
{

}

void ContextMenuHandler::showContextMenu(QWidget* widget, const QPoint &pt)
{
    // first find the parent widget
    QWidget* w = dynamic_cast<QWidget*>(parent());
    if(!w) {
        return;
    }

    // find the top level parent widget.
    QWidget* real_parent = w;
    while(real_parent->parentWidget()) {
        real_parent = real_parent->parentWidget();
    }
    // we assume that everything is managed in a graphicsview
    QGraphicsProxyWidget* proxy = real_parent->graphicsProxyWidget();
    if(proxy) {
        // use the first view for mapping the context menu to global coordinates
        auto views = proxy->scene()->views();
        auto first_view = views.first();
        QPoint gpt = first_view->mapToGlobal(first_view->mapFromScene(pt));

        // show the menu globally
        doShowContextMenu(gpt);
    }

}

void ContextMenuHandler::addHeader(QMenu &menu, const std::string &title)
{
    QAction* header = new QAction(title.c_str(), nullptr);
    header->setDisabled(true);
    QFont f = header->font();
    f.setBold(true);
    f.setUnderline(true);
    header->setFont(f);
    menu.addAction(header);
}

/// MOC
#include "../../../include/csapex/view/utility/moc_context_menu_handler.cpp"
