#ifndef CONTEXT_MENU_HANDLER_H
#define CONTEXT_MENU_HANDLER_H

/// SYSTEM
#include <QObject>
#include <QPoint>
#include <QMenu>

namespace csapex
{
class ContextMenuHandler : public QObject
{
    Q_OBJECT

public:
    virtual ~ContextMenuHandler();
    static void addHeader(QMenu& menu, const std::string& title);

public Q_SLOTS:
    void showContextMenu(const QPoint& pos);

protected:
    virtual void doShowContextMenu(const QPoint& pos) = 0;

};
}

#endif // CONTEXT_MENU_HANDLER_H
