/// HEADER
#include <csapex/view/utility/context_menu_handler.h>

/// SYSTEM
#include <QAction>

using namespace csapex;

ContextMenuHandler::~ContextMenuHandler()
{

}

void ContextMenuHandler::showContextMenu(const QPoint &pos)
{
    doShowContextMenu(pos);
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
