#ifndef CONTEXT_MENU_HANDLER_H
#define CONTEXT_MENU_HANDLER_H

/// COMPONENT
#include <csapex_qt/export.h>

/// SYSTEM
#include <QObject>
#include <QPoint>
#include <QMenu>

namespace csapex
{
class CSAPEX_QT_EXPORT ContextMenuHandler : public QObject
{
    Q_OBJECT

public:
    ~ContextMenuHandler() override;
    static void addHeader(QMenu& menu, const std::string& title);

public Q_SLOTS:
    void showContextMenu(QWidget* widget, const QPoint& pos);

protected:
    virtual void doShowContextMenu(const QPoint& pos) = 0;
};
}  // namespace csapex

#endif  // CONTEXT_MENU_HANDLER_H
