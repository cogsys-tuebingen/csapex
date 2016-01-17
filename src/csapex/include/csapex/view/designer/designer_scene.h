#ifndef DESIGNER_SCENE_H
#define DESIGNER_SCENE_H

/// COMPONENT
#include <csapex/data/point.h>
#include <csapex/command/command_fwd.h>
#include <csapex/view/view_fwd.h>
#include <csapex/view/designer/fulcrum_widget.h>
#include <csapex/view/designer/designer_styleable.h>
#include <csapex/utility/slim_signal.hpp>

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
    DesignerScene(csapex::GraphFacadePtr graph, CommandDispatcher *dispatcher, WidgetControllerPtr widget_ctrl, DesignerStyleable* style);
    ~DesignerScene();

    void drawBackground(QPainter *painter, const QRectF &rect);
    void drawForeground(QPainter *painter, const QRectF &rect);

    void mousePressEvent(QGraphicsSceneMouseEvent* e);
    void mouseMoveEvent(QGraphicsSceneMouseEvent* e);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent* e);

    int getHighlightedConnectionId() const;
    bool isEmpty() const;

    void setSelection(const NodeBox* box);
    std::vector<NodeBox*> getSelectedBoxes() const;


Q_SIGNALS:
    void eventFulcrumAdded(void *f);
    void eventFulcrumMoved(void *f, bool dropped);
    void eventFulcrumTypeChanged(void *f, int type);
    void eventFulcrumHandleMoved(void *f, bool dropped, int which);
    void eventFulcrumDeleted(void *);

    void eventConnectionAdded(Connection*);
    void evetnConnectionDeleted(Connection*);

public Q_SLOTS:
    void fulcrumAdded(void *f);
    void fulcrumMoved(void *f, bool dropped);
    void fulcrumTypeChanged(void *f, int type);
    void fulcrumHandleMoved(void *f, bool dropped, int which);
    void fulcrumDeleted(void *);


public Q_SLOTS:
    void addTemporaryConnection(Connectable* from, Connectable* to);
    void previewConnection(Connectable* from, Connectable* to);
    void addTemporaryConnection(Connectable *from, const QPointF &end);
    void deleteTemporaryConnections();
    void deleteTemporaryConnectionsAndRepaint();

    void connectionAdded(Connection*);
    void connectionDeleted(Connection*);

    bool showConnectionContextMenu();

    void refresh();
    void invalidateSchema();

    void enableGrid(bool draw);
    void enableSchema(bool draw);

    void displaySignals(bool display);
    void displayMessages(bool display);

    void enableDebug(bool debug);

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

    enum class ConnectionType {
        SIG, MSG
    };

    struct CurrentConnectionState {
        bool highlighted;
        bool error;
        bool disabled;
        bool full_read;
        bool full_unread;
        bool established;
        bool source_established;
        bool sink_established;
        bool minimized_from;
        bool minimized_to;
        bool minimized;
        bool hidden_from;
        bool hidden_to;

        ConnectionType type;

        Position start_pos;
        Position end_pos;

        QString label;

        int level;

        double r;
    };

private:
    void drawConnection(QPainter *painter, const Connection &connection);
    std::vector<QRectF> drawConnection(QPainter *painter, Connectable* from, Connectable* to, int id);
    std::vector<QRectF> drawConnection(QPainter *painter, const QPointF &from, const QPointF &to, int id);

    void drawPort(QPainter *painter, NodeBox *box, Port* p);


    QPointF offset(const QPointF& vector, Position position, double offset);

private:
    DesignerStyleable* style_;
    CurrentConnectionState ccs;

    GraphFacadePtr graph_facade_;
    CommandDispatcher* dispatcher_;
    WidgetControllerPtr widget_ctrl_;

    QPixmap background_;

    QPainter* schematics_painter;
    QImage schematics;

    MessagePreviewWidget* preview_;

    std::vector<TempConnection> temp_;

    std::vector<csapex::slim_signal::Connection> connections_;
    std::map<const Connection*,std::vector<QRectF> > connection_bb_;

    std::map<Fulcrum*,FulcrumWidget*> fulcrum_2_widget_;
    std::map<Fulcrum*,Point> fulcrum_last_pos_;
    std::map<Fulcrum*,int> fulcrum_last_type_;
    std::map<Fulcrum*,Point> fulcrum_last_hin_;
    std::map<Fulcrum*,Point> fulcrum_last_hout_;

    bool draw_grid_;
    bool draw_schema_;
    bool display_messages_;
    bool display_signals_;

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

    bool debug_;
};

}
#endif // DESIGNER_SCENE_H
