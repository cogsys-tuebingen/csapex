#ifndef DESIGNER_SCENE_H
#define DESIGNER_SCENE_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/fulcrum.h>
#include <csapex/view/fulcrum_widget.h>
#include <csapex/view/designer_styleable.h>

/// SYSTEM
#include <QGraphicsScene>
#include <QLabel>
#include <QTime>


namespace csapex
{

class DesignerScene : public QGraphicsScene
{
    Q_OBJECT

private:
    static const float ARROW_LENGTH;

public:
    DesignerScene(csapex::GraphPtr graph, CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl, DesignerStyleable* style);
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

    int getHighlightedConnectionId() const;
    bool isEmpty() const;

    std::vector<NodeBox*> getSelectedBoxes() const;

public Q_SLOTS:
    void addTemporaryConnection(Connectable* from, Connectable* to);
    void previewConnection(Connectable* from, Connectable* to);
    void addTemporaryConnection(Connectable *from, const QPointF &end);
    void deleteTemporaryConnections();
    void deleteTemporaryConnectionsAndRepaint();

    void connectionAdded(Connection*);
    void connectionDeleted(Connection*);

    void boxMoved(NodeBox* box);

    void fulcrumAdded(Fulcrum *f);
    void fulcrumMoved(Fulcrum *f, bool dropped);
    void fulcrumTypeChanged(Fulcrum *f, int type);
    void fulcrumHandleMoved(Fulcrum *f, bool dropped, int which);
    void fulcrumDeleted(Fulcrum *);

    bool showConnectionContextMenu();

    void refresh();
    void invalidateSchema();

    void enableGrid(bool draw);
    void enableSchema(bool draw);

    void setScale(double scale);

private:
    void drawGrid(const QRectF &rect, QPainter *painter, double dimension);

private:
    struct TempConnection {
        TempConnection(bool is_connected)
            : is_connected(is_connected)
        {}

        bool is_connected;

        Connectable* from;

        QPointF to_p;
        Connectable* to_c;
    };

    enum Position {
        LEFT = 0,
        RIGHT = 1,
        TOP = 2,
        BOTTOM = 3,

        UNDEFINED = 99
    };

    struct CurrentConnectionState {
        bool highlighted;
        bool error;
        bool disabled;
        bool blocked_from;
        bool blocked_to;
        bool minimized_from;
        bool minimized_to;
        bool minimized;
        bool hidden_from;
        bool hidden_to;

        Position start_pos;
        Position end_pos;

        double r;
    };

private:
    void drawConnection(QPainter *painter, const Connection &connection);
    std::vector<QRectF> drawConnection(QPainter *painter, const QPointF &from, const QPointF &to, int id);

    void drawPort(QPainter *painter, NodeBox *box, Port* p);


    QPointF offset(const QPointF& vector, Position position, double offset);

private:
    DesignerStyleable* style_;
    CurrentConnectionState ccs;

    GraphPtr graph_;
    CommandDispatcher* dispatcher_;
    WidgetControllerPtr widget_ctrl_;

    QPixmap background_;

    QPainter* schematics_painter;
    QImage schematics;

    std::vector<TempConnection> temp_;

    std::map<const Connection*,std::vector<QRectF> > connection_bb_;

    std::map<Fulcrum*,FulcrumWidget*> fulcrum_2_widget_;
    std::map<Fulcrum*,QPointF> fulcrum_last_pos_;
    std::map<Fulcrum*,int> fulcrum_last_type_;
    std::map<Fulcrum*,QPointF> fulcrum_last_hin_;
    std::map<Fulcrum*,QPointF> fulcrum_last_hout_;

    bool draw_grid_;
    bool draw_schema_;
    double scale_;
    double overlay_threshold_;


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
