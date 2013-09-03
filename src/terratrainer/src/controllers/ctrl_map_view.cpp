/// HEADER
#include "ctrl_map_view.h"
/// SYSTEM
#include <QGraphicsRectItem>
#include <QGraphicsSceneMouseEvent>
#include <QWidget>
#include <QDir>
/// COMPONENT
#include <ui_terra_trainer_window.h>

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

void CtrlMapView::write(YAML::Emitter &emitter) const
{
    emitter << YAML::Key << "ROIS" << YAML::Value;
    emitter << YAML::BeginSeq;
    QList<QGraphicsItem*> items = map_view_scene_->selectedItems();
    foreach(QGraphicsItem *item, items) {
        QInteractiveItem *roi = dynamic_cast<QInteractiveItem*>(item);

        if(roi != NULL) {
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "class" << YAML::Value << roi->getClass();
            QPointF pos  = roi->scenePos();
            QRectF  geom = roi->rect();
            emitter << YAML::Key << "x" << YAML::Value << pos.x();
            emitter << YAML::Key << "y" << YAML::Value << pos.y();
            emitter << YAML::Key << "w" << YAML::Value << geom.width();
            emitter << YAML::Key << "h" << YAML::Value << geom.height();
            emitter << YAML::EndMap;
        }
    }

    emitter << YAML::EndSeq;
}

void CtrlMapView::read(const YAML::Node &document)
{
    if(map_view_ == NULL) {
        std::cerr << "Mapview wasn't initialized so far!" << std::endl;
        return;
    }

    map_view_->setEnabled(false);

    if(!imageUpdate(true)) {
        std::cerr << "Couldn't load rois on empty image!" << std::endl;
        return;
    }

    try {
        const YAML::Node &colors = document["ROIS"];
        for(YAML::Iterator it = colors.begin() ; it != colors.end(); it++) {
            double x,y,w,h;
            int id;
            (*it)["class"] >> id;
            (*it)["x"] >> x;
            (*it)["y"] >> y;
            (*it)["w"] >> w;
            (*it)["h"] >> h;

            changeClass(id);
            QPointF pos;
            pos.setX(x);
            pos.setY(y);
            addRectangle(pos, w, h);
        }
    } catch (YAML::Exception e) {
        std::cerr << "Class Editor cannot read config : '" << e.what() <<"' !" << std::endl;
    }

    map_view_->setEnabled(true);
}

bool CtrlMapView::imageUpdate(bool cached)
{
    /// CLEAR
    clearAll();

    if(!cached) {
        cache_ = bridge_->rawImage();
        initGUI();
    }

    if(cache_.get() == NULL)
        return false;


    map_image_ = new QGraphicsPixmapItem(QPixmap::fromImage(*cache_));
    map_view_scene_->setBackground(map_image_);
    map_view_->show();
    return true;
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
    QColor color = bridge_->colorGet(id);
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
    QColor color  = bridge_->colorGet(id);
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

        if(m->type() == QEvent::GraphicsSceneMousePress && m->button() == Qt::LeftButton
                || mouse_move_) {
            mouse_move_ = true;
            if(map_view_scene_->getMode() == QInteractiveScene::ADD){
                addRectangle(m->scenePos() - QPointF(box_size_ / 2.0, box_size_ / 2.0), box_size_, box_size_);
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
    Q_EMIT computeROIS();

    /// bridge_->compute();
}

void CtrlMapView::computeQuad()
{
    if(bridge_->recalcQuad())
        Q_EMIT computeQUAD();
        //bridge_->computeQuadtree();
    else
        loadOverlay(QUAD, rendered_quad_);
}

void CtrlMapView::computeGrid()
{
    if(bridge_->recalcGrid())
        Q_EMIT computeGRID();
        //       bridge_->computeGrid();
    else
        loadOverlay(GRID, rendered_grid_);
}

void CtrlMapView::computeFinished()
{
    std::cout << "Finished computation!" << std::endl;
}

void CtrlMapView::computeGridFinished()
{
    renderGrid();
    loadOverlay(GRID, rendered_grid_);
}

void CtrlMapView::computeQuadFinished()
{
    renderQuad();
    loadOverlay(QUAD, rendered_quad_);
}

void CtrlMapView::saveSelectedCrops(QString path)
{
    /// PREPARE FOLDERS
    std::vector<int> registered_classes = bridge_->getClassIDs();
    foreach(int id , registered_classes) {

        QString id_path = path + "/" + QString::number(id);
        QDir directory(id_path);
        if(!directory.exists() && !QDir().mkdir(id_path)) {
            std::cerr << "Folder cannot be created '" << id_path.toUtf8().constData() << "' !" << std::endl;
            return;
        }
    }
    std::map<int, int> counts;
    QList<QGraphicsItem*> rects = map_view_scene_->selectedItems();
    foreach (QGraphicsItem* item, rects) {
        QInteractiveItem *rect = dynamic_cast<QInteractiveItem*>(item);

        if(rect != NULL) {
            /// COUNTS
            int id = rect->getClass();
            if(counts.find(id) != counts.end())
                counts[id]++;
            else
                counts.insert(std::pair<int,int>(id, 0));

            QString file_path = path + "/" + QString::number(id) + "/" + QString::number(counts[id]) + ".jpg";
            QRectF crop_rectf = rect->rect();
            QRect  crop_rect(crop_rectf.x(), crop_rectf.y(), crop_rectf.width(), crop_rectf.height());
            QImage  crop = cache_->copy(crop_rect);
            crop.save(file_path);
        }
    }
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

    QInteractiveItem *rect = new QInteractiveItem(QRectF(0, 0, width, height));
    rect->setPen(current_class_pen_);
    rect->setSelectPen(current_class_sel_pen_);
    rect->setClass(current_class_id_);
    map_view_scene_->addInteractive(rect);
    rect->setLimitedPos(pos);
}

void CtrlMapView::removeItem(const QPointF &pos)
{
    QGraphicsItem *to_remove = map_view_scene_->itemAt(pos);
    if(to_remove != NULL)
        map_view_scene_->removeInteractive(to_remove);
}

void CtrlMapView::renderGrid()
{
    /// CLEANUP
    clearRenderedLayer(rendered_grid_);

    /// RENDER
    std::vector<cv_roi::TerraROI> grid;
    bridge_->getGrid(grid);
    for(std::vector<cv_roi::TerraROI>::iterator it = grid.begin() ; it != grid.end() ; it++) {
        rendered_grid_.push_back(renderBox(*it));
    }

    std::cout << "INFO : " << rendered_grid_.size() << " grid cells have been rendered." << std::endl;
}

void CtrlMapView::renderQuad()
{
    /// CLEANUP
    clearRenderedLayer(rendered_quad_);

    /// RENDER
    std::vector<cv_roi::TerraROI> tree;
    bridge_->getQuadtree(tree);
    for(std::vector<cv_roi::TerraROI>::iterator it = tree.begin() ; it != tree.end() ; it++) {
        rendered_quad_.push_back(renderBox(*it));
    }

    std::cout << "INFO : " << rendered_quad_.size() << " quad tree regions have been rendered." << std::endl;
}

void CtrlMapView::loadOverlay(const Overlay overlay, const QInteractiveScene::Layer &layer)
{
    if(overlay_ != overlay) {
        map_view_scene_->clearOverlay();
        map_view_scene_->setInteractiveVisible(false);
        map_view_scene_->setOverlay(layer);
        overlay_ = overlay;

        if(rendered_grid_.size() == 0) {
            std::cout << "Set empty layer!" << std::endl;
        }
    } else {
        map_view_scene_->clearOverlay();
        map_view_scene_->setInteractiveVisible(true);
        overlay_ = NONE;
    }
}

void CtrlMapView::unloadOverlay()
{
    map_view_scene_->clearOverlay();
    map_view_scene_->setInteractiveVisible(true);
    overlay_ = NONE;
}

QGraphicsRectItem *CtrlMapView::renderBox(cv_roi::TerraROI &roi)
{
    QPen pen;
    pen.setColor(bridge_->colorGet(roi.id.id));
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

void CtrlMapView::clearRenderedLayer(QInteractiveScene::Layer &rendered)
{

    //// TODO : DELETE THE ITEMS
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
    clearRenderedLayer(rendered_grid_);
    clearRenderedLayer(rendered_quad_);
    map_view_scene_->clearOverlay();
}
