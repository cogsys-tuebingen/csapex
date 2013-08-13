/// HEADER
#include "ctrl_map_view.h"
#include <QGraphicsRectItem>
#include <QGraphicsSceneMouseEvent>

/// SYSTEM

CtrlMapView::CtrlMapView(QGraphicsView *mapView, CMPCoreBridge::Ptr bridge) :
    map_view_(mapView),
    map_view_scene_(new QInteractiveScene(map_view_)),
    map_image_(NULL),
    bridge_(bridge),
    zoom_(100.0),
    box_size_(20.0),
    current_class_id_(-1)
{
    initGUI();
}

CtrlMapView::~CtrlMapView()
{
}

void CtrlMapView::imageUpdate(bool cached)
{
    map_view_scene_->clearAll();

    if(!cached)
        cache_ = bridge_->rawImage();

    if(cache_.get() == NULL)
        return;

    map_image_ = new QGraphicsPixmapItem(QPixmap::fromImage(*cache_));
    map_view_scene_->setBackground(map_image_);
    map_view_->show();
}

void CtrlMapView::zoom(double factor)
{
    zoom_ = std::min(MAX_ZOOM, std::max(MIN_ZOOM, factor));
    double scale = zoom_ * 0.01;
    map_view_->resetMatrix();
    map_view_->setTransformationAnchor(QGraphicsView::AnchorViewCenter);
    map_view_->scale(scale, scale);

    Q_EMIT zoomUpdated(zoom_);
}

void CtrlMapView::size(double size)
{
    box_size_ = size;
    Q_EMIT sizeUpdated(size);
}

void CtrlMapView::activateAdd()
{
    map_view_scene_->setMode(QInteractiveScene::ADD);
}

void CtrlMapView::activateMove()
{
    map_view_scene_->setMode(QInteractiveScene::MOVE);
}

void CtrlMapView::activateDelete()
{
    map_view_scene_->setMode(QInteractiveScene::REMOVE);
}

void CtrlMapView::activateSelect()
{
    map_view_scene_->setMode(QInteractiveScene::SELECT);
}

void CtrlMapView::activateTrash()
{
    imageUpdate(true);
}

void CtrlMapView::selectAll()
{
    map_view_scene_->selectAll();
}

void CtrlMapView::deselectAll()
{
    map_view_scene_->deselectAll();
}

void CtrlMapView::changeClass(int id)
{
    current_class_id_ = id;
    QColor color = bridge_->getColorByClass(id);
    current_class_pen_.setColor(color);
    current_class_sel_pen_.setColor(color);
}

void CtrlMapView::classRemoved(int id)
{
    if(current_class_id_ = id)
        current_class_id_ = -1;

    QList<QGraphicsItem*> items = map_view_scene_->interactive();
    foreach (QGraphicsItem *item, items) {
        QInteractiveItem *interactive = dynamic_cast<QInteractiveItem*>(item);
        if(interactive != NULL) {
            if(interactive->getClass() == id) {
                map_view_scene_->removeItem(interactive);
            }
        }
    }
}

void CtrlMapView::classUpdated(int oldID, int newID)
{
    if(current_class_id_ == oldID)
        current_class_id_ = newID;

    QList<QGraphicsItem*> items = map_view_scene_->interactive();
    foreach (QGraphicsItem *item, items) {
        QInteractiveItem *interactive = dynamic_cast<QInteractiveItem*>(item);
        if(interactive != NULL) {
            if(interactive->getClass() == oldID) {
                interactive->setClass(newID);
            }
        }
    }
}

void CtrlMapView::colorUpdate(int id)
{
    QColor color  = bridge_->getColorByClass(id);
    if(current_class_id_ = id) {
        current_class_pen_.setColor(color);
        current_class_sel_pen_.setColor(color);
    }


    QList<QGraphicsItem*> items = map_view_scene_->interactive();
    QPen   defPen = current_class_pen_;
    QPen   selPen = current_class_sel_pen_;
    defPen.setColor(color);
    selPen.setColor(color);

    foreach (QGraphicsItem *item, items) {
        QInteractiveItem *interactive = dynamic_cast<QInteractiveItem*>(item);
        if(interactive != NULL) {
            if(interactive->getClass() == id) {
                interactive->setPen(defPen);
                interactive->setSelectPen(selPen);
            }
        }
    }

    map_view_->repaint();
}

bool CtrlMapView::eventFilter(QObject *obj, QEvent *event)
{
    QGraphicsSceneMouseEvent *m = dynamic_cast<QGraphicsSceneMouseEvent*>(event);
    if(m != NULL && map_image_ != NULL) {
        if(m->type() == QEvent::GraphicsSceneMousePress) {
            if(m->button() == Qt::LeftButton) {
                if(map_view_scene_->getMode() == QInteractiveScene::ADD){
                    addRectangle(m->scenePos(), box_size_, box_size_);
                }
                if(map_view_scene_->getMode() == QInteractiveScene::REMOVE){
                    removeItem(m->scenePos());
                }
                event->accept();
            }
        }
    }

    map_view_->show();
    return QObject::eventFilter(obj, event);

}

void CtrlMapView::compute()
{
    std::vector<cv_roi::TerraROI> to_compute;

    QList<QGraphicsItem*>  items = map_view_scene_->selectedItems();
    foreach(QGraphicsItem* item, items) {
        QInteractiveItem* ptr = dynamic_cast<QInteractiveItem*>(item);
        if(ptr != NULL) {
            cv_roi::TerraROI roi;
            QRectF bounding = ptr->rect();
            roi.roi.rect.x      = std::floor(bounding.x() + 0.5);
            roi.roi.rect.y      = std::floor(bounding.y() + 0.5);
            roi.roi.rect.width  = std::floor(bounding.width() + 0.5);
            roi.roi.rect.height = std::floor(bounding.height() + 0.5);
            roi.roi.rotation    = 0.0;
            roi.id.id           = ptr->getClass();
            to_compute.push_back(roi);
        }

    }

    bridge_->compute(to_compute);
}

void CtrlMapView::trainingFinished()
{

}

void CtrlMapView::feedbackFinished()
{

}

void CtrlMapView::initGUI()
{
    /// PENS 'N' BRUSHES
    DarkBrush.setStyle(Qt::SolidPattern);
    DarkBrush.setColor(QColor(50,50,50));

    current_class_pen_.setWidth(1);
    current_class_pen_.setCosmetic(true);
    current_class_sel_pen_.setWidth(1);
    current_class_sel_pen_.setCosmetic(true);
    current_class_sel_pen_.setStyle(Qt::DashLine);
    current_class_sel_pen_.setDashOffset(3.0);

    /// GUI
    map_view_scene_->installEventFilter(this);
    map_view_->setAcceptDrops(true);
    map_view_->setBackgroundBrush(DarkBrush);
    map_view_->setScene(map_view_scene_);
    map_view_->fitInView(map_view_scene_->itemsBoundingRect(), Qt::KeepAspectRatio);

}

QInteractiveItem *CtrlMapView::addRectangle(const QPointF pos, const qreal width, const qreal height)
{
    if(current_class_id_ == -1)
        return NULL;

    QInteractiveItem *rect = new QInteractiveItem(QRectF(pos.x() - 10, pos.y() - 10, width, height));
    rect->setPen(current_class_pen_);
    rect->setSelectPen(current_class_sel_pen_);
    rect->setClass(current_class_id_);
    map_view_scene_->addInteractive(rect);
}

void CtrlMapView::removeItem(const QPointF &pos)
{
    QGraphicsItem *to_remove = map_view_scene_->itemAt(pos);
    if(to_remove != NULL)
        map_view_scene_->removeInteractive(to_remove);
}

void CtrlMapView::renderGrid()
{
    std::vector<cv_roi::TerraROI> grid;
    bridge_->getGrid(grid);

}

void CtrlMapView::renderTree()
{
    std::vector<cv_roi::TerraROI> tree;
    bridge_->getQuadtree(tree);

}
