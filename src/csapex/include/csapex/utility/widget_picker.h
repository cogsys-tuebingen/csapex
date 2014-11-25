#ifndef WIDGET_PICKER_H
#define WIDGET_PICKER_H

/// PROJECT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QObject>
#include <QMutex>
#include <QWaitCondition>

namespace csapex
{
class WidgetPicker : public QObject
{
    Q_OBJECT

public:
    WidgetPicker();

    void startPicking(DesignerScene* designer_scene);

    QWidget* getWidget();

    bool eventFilter(QObject *, QEvent *);

Q_SIGNALS:
    void widgetPicked();

private:
    DesignerScene* designer_scene_;

    QWidget* widget_;
};
}

#endif // WIDGET_PICKER_H
