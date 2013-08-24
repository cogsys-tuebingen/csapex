#ifndef CTRL_MAP_VIEW_H
#define CTRL_MAP_VIEW_H

/// COMPONENT
#include <graphics/qinteractive_rect.h>
#include <graphics/qinteractive_scene.h>
#include "ctrl_cmpcore_bridge.h"
#include "controller.hpp"

/// SYSTEM
#include <QObject>
#include <QGraphicsView>
#include <QGraphicsRectItem>

namespace Ui {
class TerraTrainerWindow;
}

static const double MAX_ZOOM(200.0);
static const double MIN_ZOOM(0.25);

class CtrlMapView : public QObject, public Controller
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<CtrlMapView> Ptr;

    CtrlMapView(CMPCoreBridge::Ptr bridge);
    virtual ~CtrlMapView();

    void setupUI(Ui::TerraTrainerWindow *ui);
    void write(YAML::Emitter &emitter) const;
    void read(const YAML::Node &document);

    void saveSelectedCrops(QString path);


Q_SIGNALS:
    void zoomUpdated(double factor);
    void sizeUpdated(double size);
    void computeROIS();
    void computeGRID();
    void computeQUAD();

public Q_SLOTS:
    /// GRAPHICAL INTERACTION
    bool imageUpdate(bool cached = false);
    void zoom(double factor);
    void size(double size);

    /// INPUT SLOTS FOR GUI INTERACTION
    void activateAdd();
    void activateMove();
    void activateDelete();
    void activateSelect();
    void activateTrash();
    void selectAll();
    void deselectAll();
    void changeClass(int id);

    void classRemoved(int id);
    void classUpdated(int oldID, int newID);
    void colorUpdate(int id);

    /// INVOKE COMPUTATION
    void compute();
    void computeQuad();
    void computeGrid();
    void computeFinished();
    void computeGridFinished();
    void computeQuadFinished();


protected:
    bool eventFilter(QObject *obj, QEvent *event);

private:
    enum Overlay {NONE, GRID, QUAD};
    Overlay                   overlay_;

    QWidget                  *central_widget_;
    QGraphicsView            *map_view_;
    QInteractiveScene        *map_view_scene_;
    QGraphicsPixmapItem      *map_image_;
    QInteractiveScene::Layer  rendered_quad_;
    QInteractiveScene::Layer  rendered_grid_;
    boost::shared_ptr<QImage> cache_;
    CMPCoreBridge::Ptr        bridge_;
    double                    zoom_;
    double                    min_zoom_;
    double                    box_size_;

    QPen                      current_class_pen_;
    QPen                      current_class_sel_pen_;
    int                       current_class_id_;

    QBrush                    DarkBrush;
    bool                      mouse_move_;

    void setCurrentSelectedROIs();

    void initGUI();
    QInteractiveItem  *addRectangle(const QPointF pos, const qreal width, const qreal height);
    void removeItem(const QPointF &pos);

    void renderGrid();
    void renderQuad();
    void loadOverlay(const Overlay overlay, const QInteractiveScene::Layer &layer);
    void unloadOverlay();

    QGraphicsRectItem* renderBox(cv_roi::TerraROI &roi);

    void clearRenderedLayer(QInteractiveScene::Layer &rendered);
    void clearAll();
    void clearOverlay();
};

#endif // CTRL_MAP_VIEW_H
