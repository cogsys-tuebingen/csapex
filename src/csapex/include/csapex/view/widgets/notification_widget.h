#ifndef NOTIFICATION_WIDGET_H
#define NOTIFICATION_WIDGET_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/utility/notification.h>

/// PROJECT
#include <csapex/view/view_fwd.h>

/// SYSTEM
#include <QWidget>

class QLabel;
class QTimer;
class QGraphicsOpacityEffect;

namespace csapex
{

class CSAPEX_QT_EXPORT NotificationWidget : public QWidget
{
    Q_OBJECT

public:
    NotificationWidget(const Notification& notification, QWidget* parent = nullptr);
    ~NotificationWidget();

    void setNotification(const Notification& notification);

    bool eventFilter(QObject *, QEvent *);

Q_SIGNALS:
    void activated(AUUID id);
    void timeout();

private Q_SLOTS:
    void fadeout();
    void shutdown();

protected:
    void paintEvent(QPaintEvent* e);
    void mouseReleaseEvent(QMouseEvent* me);

    void updateNotification();

private:
    QTimer* timer_;
    QGraphicsOpacityEffect *eff;

    Notification notification_;

    QLabel* icon_;
    QLabel* label_;
};

}
#endif // NOTIFICATION_WIDGET_H
