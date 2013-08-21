/// HEADER
#include "ctrl_map_view.h"
#include <QGraphicsRectItem>
#include <QGraphicsSceneMouseEvent>
#include <QWidget>
#include <ui_terra_trainer_window.h>
/// SYSTEM

CtrlMapView::CtrlMapView(CMPCoreBridge::Ptr bridge) :
    overlay_(NONE),
    map_view_(NULL),
    map_view_scene_(NULL),
    map_image_(NULL),
    bridge_(bridge),
    zoom_(100.0),
    box_size_(20.0),
    current_class_id_(-1),
    mouse_move_(false)
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

}

CtrlMapView::~CtrlMapView()
{
}

void CtrlMapView::setupUI(Ui::TerraTrainerWindow *ui)
{
    central_widget_ = ui->centralWidget;
    map_view_       = ui->mapView;

    /// INIT GUI
    initGUI();
}

void CtrlMapView::imageUpdate(bool cached)
{
    /// CLEAR
    clearAll();

    if(!cached) {
        cache_ = bridge_->rawImage();
        initGUI();
    }

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
    clearOverlay();
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
    clearOverlay();
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
    clearOverlay();
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
    clearOverlay();
}

bool CtrlMapView::eventFilter(QObject *obj, QEvent *event)
{
    QGraphicsSceneMouseEvent *m = dynamic_cast<QGraphicsSceneMouseEvent*>(event);
    if(m != NULL && map_image_ != NULL && overlay_ == NONE) {
        if(m->type() == QEvent::GraphicsSceneMouseRelease)
            mouse_move_ = false;

        if(m->type() == QEvent::GraphicsSceneMousePress && m->button() == Qt::LeftButton || mouse_move_) {
            mouse_move_ = true;
            if(map_view_scene_->getMode() == QInteractiveScene::ADD){
                addRectangle(m->scenePos(), box_size_, box_size_);
            }
            if(map_view_scene_->getMode() == QInteractiveScene::REMOVE){
                removeItem(m->scenePos());
            }
            event->accept();

        }
    }

    map_view_->show();
    return QObject::eventFilter(obj, event);

}

void CtrlMapView::compute()
{
    /// CLEAR THE OLD OVERLAY
    clearOverlay();
    setCurrentSelectedROIs();
    bridge_->compute();
}

void CtrlMapView::computeQuad()
{
    bridge_->computeQuadtree();
}

void CtrlMapView::computeGrid()
{
    bridge_->computeGrid();
}

void CtrlMapView::computeFinished()
{
    std::cout << "Finished computation!" << std::endl;
}

void CtrlMapView::computeGridFinished()
{
    renderGrid();
    if(overlay_ != GRID) {
        map_view_scene_->clearOverlay();
        if(rendered_grid_.size() > 0) {
            map_view_scene_->setInteractiveVisible(false);
            map_view_scene_->setOverlay(rendered_grid_);
            overlay_ = GRID;
        } else {
            std::cerr << "No grid rendered so far!" << std::endl;
        }
    } else {
        map_view_scene_->clearOverlay();
        map_view_scene_->setInteractiveVisible(true);
        overlay_ = NONE;
    }
}

void CtrlMapView::computeQuadFinished()
{
    renderTree();
    if(overlay_ != QUAD) {
        map_view_scene_->clearOverlay();
        if(rendered_tree_.size() > 0) {
            map_view_scene_->setInteractiveVisible(false);
            map_view_scene_->setOverlay(rendered_tree_);
            overlay_ = QUAD;
        } else {
            std::cerr << "No quadtree segmentation rendered so far!" << std::endl;
        }
    } else {
        map_view_scene_->clearOverlay();
        map_view_scene_->setInteractiveVisible(true);
        overlay_ = NONE;
    }
}

void CtrlMapView::saveROIs(QString path)
{
    setCurrentSelectedROIs();
    bridge_->saveROIs(path);
}

void CtrlMapView::setCurrentSelectedROIs()
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
    bridge_->setROIs(to_compute);
}

void CtrlMapView::initGUI()
{

    /// CLEAN UP
    if(map_view_scene_ != NULL)
        map_view_scene_->deleteLater();

    if(map_view_ != NULL)
        map_view_->deleteLater();

    /// ...
    map_view_       = new QGraphicsView();
    map_view_scene_ = new QInteractiveScene(map_view_);

    central_widget_->layout()->addWidget(map_view_);


    /// GUI
    map_view_scene_->installEventFilter(this);
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
    /// RENDER
    for(std::vector<cv_roi::TerraROI>::iterator it = grid.begin() ; it != grid.end() ; it++) {
        rendered_grid_.push_back(renderBox(*it));
    }

    std::cout << "INFO : " << rendered_grid_.size() << " grid cells have been rendered." << std::endl;
}

void CtrlMapView::renderTree()
{
    map_view_scene_->clearOverlay();
    std::vector<cv_roi::TerraROI> tree;
    bridge_->getQuadtree(tree);

    /// RENDER
    for(std::vector<cv_roi::TerraROI>::iterator it = tree.begin() ; it != tree.end() ; it++) {
        rendered_tree_.push_back(renderBox(*it));
    }

    std::cout << "INFO : " << rendered_tree_.size() << " quad tree regions have been rendered." << std::endl;
}

QGraphicsRectItem *CtrlMapView::renderBox(cv_roi::TerraROI &roi)
{
    QPen pen;
    pen.setColor(bridge_->getColorByClass(roi.id.id));
    QRectF rect(roi.roi.rect.x,
                roi.roi.rect.y,
                roi.roi.rect.width,
                roi.roi.rect.height);

    QGraphicsRectItem *rectItem = new QGraphicsRectItem(rect);
    rectItem->setData(0, QVariant(roi.id.id));
    rectItem->setPen(pen);
    rectItem->setToolTip(QString::number(roi.id.prob));

    return rectItem;
}

void CtrlMapView::clearRendered(QInteractiveScene::Layer &rendered)
{
    rendered.clear();
}

void CtrlMapView::clearAll()
{
    map_view_scene_->clearAll();
    clearOverlay();
}

void CtrlMapView::clearOverlay()
{
    overlay_ = NONE;
    clearRendered(rendered_grid_);
    clearRendered(rendered_tree_);
    map_view_scene_->clearOverlay();
}
