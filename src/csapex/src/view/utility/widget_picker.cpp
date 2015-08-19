/// HEADER
#include <csapex/view/utility/widget_picker.h>

/// PROJECT
#include <csapex/view/designer/designer_scene.h>

/// SYSTEM
#include <QEvent>
#include <QGraphicsSceneMouseEvent>
#include <iostream>
#include <QWidget>
#include <QGraphicsProxyWidget>
#include <QApplication>
#include <assert.h>

using namespace csapex;

WidgetPicker::WidgetPicker()
    : designer_scene_(nullptr), widget_(nullptr)
{

}

void WidgetPicker::startPicking(DesignerScene* designer_scene)
{
    assert(designer_scene);

    designer_scene_ = designer_scene;
    designer_scene_->installEventFilter(this);

    QApplication::setOverrideCursor(Qt::CrossCursor);
}

QWidget* WidgetPicker::getWidget()
{
    designer_scene_->removeEventFilter(this);
    QApplication::restoreOverrideCursor();
    designer_scene_ = nullptr;

    return widget_;
}

bool WidgetPicker::eventFilter(QObject*, QEvent * e)
{
    switch(e->type()) {
    case QEvent::KeyPress: {
        QKeyEvent* ke = dynamic_cast<QKeyEvent*>(e);
        int key = ke->key();

        if(key == Qt::Key_Escape) {
            e->accept();
            widget_ = nullptr;

            Q_EMIT widgetPicked();
            return true;
        }
    }
        break;

    case QEvent::GraphicsSceneMousePress: {
        QGraphicsSceneMouseEvent* me = dynamic_cast<QGraphicsSceneMouseEvent*> (e);
        if(me->button() == Qt::LeftButton) {
            QGraphicsItem* item = designer_scene_->itemAt(me->scenePos(), QTransform());

            if(item && item->type() == QGraphicsProxyWidget::Type) {
                QGraphicsProxyWidget* proxy = static_cast<QGraphicsProxyWidget*>(item);
                QWidget* widget = proxy->widget();
                QPointF p = proxy->mapFromScene(me->scenePos());
                QWidget* child = widget->childAt(p.toPoint());

                widget_ = child;
            } else {
                widget_ = nullptr;
            }

            Q_EMIT widgetPicked();

            e->accept();
            return true;
        }
    }
        break;

    default:
        break;
    }

    return false;
}
/// MOC
#include "../../../include/csapex/view/utility/moc_widget_picker.cpp"
