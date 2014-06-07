#ifndef DESIGNER_SCENE_H
#define DESIGNER_SCENE_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/connection.h>
#include <csapex/model/fulcrum.h>
#include <csapex/view/fulcrum_widget.h>

/// SYSTEM
#include <QGraphicsScene>
#include <QLabel>
#include <QTime>


namespace csapex
{

class DesignerScene : public QGraphicsScene
{
    Q_OBJECT

public:
    DesignerScene(csapex::GraphPtr graph, CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl);
    ~DesignerScene();

    void drawBackground(QPainter *painter, const QRectF &rect);
    void drawForeground(QPainter *painter, const QRectF &rect);
    void drawItems(QPainter *painter, int numItems,
                           QGraphicsItem *items[],
                           const QStyleOptionGraphicsItem options[],
                           QWidget *widget = 0);

    void mousePressEvent(QGraphicsSceneMouseEvent* e);
    void mouseMoveEvent(QGraphicsSceneMouseEvent* e);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent* e);



public Q_SLOTS:
    void addTemporaryConnection(Connectable* from, Connectable* to);
    void addTemporaryConnection(Connectable *from, const QPointF &end);
    void deleteTemporaryConnections();
    void deleteTemporaryConnectionsAndRepaint();

    void connectionAdded(Connection*);
    void connectionDeleted(Connection*);

    void fulcrumAdded(Fulcrum *f);
    void fulcrumMoved(Fulcrum *f, bool dropped);
    void fulcrumDeleted(Fulcrum *);

//    bool showConnectionContextMenu(const QPoint& pos);
//    bool showFulcrumContextMenu(const QPoint& pos);

    void refresh();
    void invalidateSchema();

    void enableGrid(bool draw);
    void enableSchema(bool draw);

    void setScale(double scale);

    void setInputColor(const QColor& c)
    {
        input_color_ = c;
    }
    void setOutputColor(const QColor& c)
    {
        output_color_ = c;
    }

private:
    void drawConnection(QPainter *painter, Connection& connection);
    void drawConnection(QPainter *painter, const QPointF &from, const QPointF &to, int id, Fulcrum::Type from_type, Fulcrum::Type to_type);

    void drawActivity(QPainter *painter, int life, Connectable* c);
    void drawPort(QPainter *painter, NodeBox *box, Port* p);

    QPen makeLinePen(const QPointF &from, const QPointF &to);
    QPen makeSelectedLinePen(const QPointF &from, const QPointF &to);

private:
    struct TempConnection {
        Connectable* from;
        QPointF to;
    };

    struct CurrentConnectionState {
        bool selected;
        bool highlighted;
        bool error;
        bool disabled;
        bool async;
        bool minimized_from;
        bool minimized_to;
        bool minimized;

        double r;
    };

    QColor output_color_;
    QColor input_color_;


    CurrentConnectionState ccs;

    GraphPtr graph_;
    CommandDispatcher* dispatcher_;
    WidgetControllerPtr widget_ctrl_;

    QPixmap background_;

    QPainter* schematics_painter;
    QImage schematics;

    std::vector<TempConnection> temp_;

    std::map<Fulcrum*,FulcrumWidget*> fulcrum_2_widget_;
    std::map<Fulcrum*,QPointF> last_pos_;

    bool draw_grid_;
    bool draw_schema_;
    double scale_;


    int activity_marker_min_width_;
    int activity_marker_max_width_;
    int activity_marker_min_opacity_;
    int activity_marker_max_opacity_;

    int highlight_connection_id_;
    int highlight_connection_sub_id_;

    int connector_radius_;

    bool schema_dirty_;
};

}
#endif // DESIGNER_SCENE_H
