#ifndef DESIGNER_SCENE_H
#define DESIGNER_SCENE_H

/// COMPONENT
#include <csapex_qt/export.h>
#include <csapex/data/point.h>
#include <csapex/command/command_fwd.h>
#include <csapex/view/view_fwd.h>
#include <csapex/view/designer/fulcrum_widget.h>
#include <csapex/view/designer/designer_styleable.h>
#include <csapex/view/csapex_view_core.h>
#include <csapex/utility/slim_signal.hpp>
#include <csapex/utility/uuid.h>
#include <csapex/profiling/profilable.h>

/// SYSTEM
#include <QGraphicsScene>
#include <QLabel>
#include <QTime>
#include <unordered_map>
#include <QPointer>

namespace csapex
{
class ConnectionDescription;
class Timer;

class CSAPEX_QT_EXPORT DesignerScene : public QGraphicsScene, public Profilable
{
    Q_OBJECT

private:
    static const float ARROW_LENGTH;

public:
    DesignerScene(csapex::GraphFacadePtr graph, CsApexViewCore& view_core);
    ~DesignerScene() override;

    void drawBackground(QPainter* painter, const QRectF& rect) override;
    void drawForeground(QPainter* painter, const QRectF& rect) override;

    void mousePressEvent(QGraphicsSceneMouseEvent* e) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent* e) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent* e) override;

    int getHighlightedConnectionId() const;
    bool isEmpty() const;

    void setSelection(const NodeBox* box);
    std::vector<NodeBox*> getSelectedBoxes() const;

    void addPort(Port* port);
    Port* getPort(const UUID& connector_uuid);
    void removePort(Port* port);

    std::string makeStatusString() const;

public Q_SLOTS:
    void fulcrumAdded(Fulcrum* f);
    void fulcrumMoved(void* f, bool dropped);
    void fulcrumTypeChanged(void* f, int type);
    void fulcrumHandleMoved(void* f, bool dropped, int which);
    void fulcrumDeleted(void*);

public Q_SLOTS:
    void addTemporaryConnection(ConnectorPtr from, ConnectorPtr to);
    void previewConnection(ConnectorPtr from, ConnectorPtr to);
    void addTemporaryConnection(ConnectorPtr from, const QPointF& end);
    void deleteTemporaryConnections();
    void deleteTemporaryConnectionsAndRepaint();

    void connectionAdded(const ConnectionDescription& ci);
    void connectionDeleted(const ConnectionDescription& ci);

    bool showConnectionContextMenu();

    void refresh();
    void invalidateSchema();

    void enableGrid(bool draw);
    void enableSchema(bool draw);

    void displaySignals(bool display);
    void displayMessages(bool display);
    void displayActive(bool display);
    void displayInactive(bool display);

    void displayConnections(const QString& key, bool show);

    void enableDebug(bool debug);

    void setScale(double scale);

private:
    void drawGrid(const QRectF& rect, QPainter* painter, double dimension);

    void displayConnections(bool& member, bool show);

private:
    struct TempConnection
    {
        TempConnection(bool is_connected) : is_connected(is_connected)
        {
        }

        bool is_connected;

        ConnectorWeakPtr from;

        QPointF to_p;
        ConnectorWeakPtr to_c;
    };

    enum Position
    {
        LEFT = 0,
        RIGHT = 1,
        TOP = 2,
        BOTTOM = 3,

        UNDEFINED = 99
    };

    enum class TokenType
    {
        SIG,
        MSG
    };

    struct CurrentConnectionState
    {
        bool highlighted;
        bool error;
        bool disabled;
        bool marker_token;
        bool full_read;
        bool full_unread;
        bool minimized_from;
        bool minimized_to;
        bool minimized;
        bool hidden_from;
        bool hidden_to;
        bool selected_from;
        bool selected_to;
        bool active;
        bool active_token;

        bool target_is_pipelining;

        TokenType type;

        Position start_pos;
        Position end_pos;

        QString label;

        double r;
    };

private:
    void drawConnection(QPainter* painter, const ConnectionDescription& connection);
    std::vector<QRectF> drawConnection(QPainter* painter, Connector* from, Connector* to, int id);
    std::vector<QRectF> drawConnection(QPainter* painter, const QPointF& from, const QPointF& to, int id);

    void drawPort(QPainter* painter, bool selected, Port* p, int pos = -1);

    QPointF offset(const QPointF& vector, Position position, double offset);

    void showPreview();

private:
    CsApexViewCore& view_core_;

    CurrentConnectionState ccs;

    GraphFacadePtr graph_facade_;

    QPixmap background_;

    QPainter* schematics_painter;
    QImage schematics;

    QTimer* preview_timer_;
    MessagePreviewWidget* preview_;

    std::vector<TempConnection> temp_;

    std::vector<csapex::slim_signal::Connection> connections_;
    std::map<int, std::vector<QRectF>> connection_bb_;

    std::map<int, std::vector<Fulcrum>> connection_2_fulcrum_;
    std::map<Fulcrum*, FulcrumWidget*> fulcrum_2_widget_;
    std::map<Fulcrum*, Point> fulcrum_last_pos_;
    std::map<Fulcrum*, int> fulcrum_last_type_;
    std::map<Fulcrum*, Point> fulcrum_last_hin_;
    std::map<Fulcrum*, Point> fulcrum_last_hout_;

    bool draw_grid_;
    bool draw_schema_;
    bool display_messages_;
    bool display_signals_;
    bool display_active_;
    bool display_inactive_;

    double scale_;

    int activity_marker_min_width_;
    int activity_marker_max_width_;
    int activity_marker_min_opacity_;
    int activity_marker_max_opacity_;

    int highlight_connection_id_;
    int highlight_connection_sub_id_;

    int connector_radius_;

    bool schema_dirty_;

    bool debug_;

    std::unordered_map<UUID, QPointer<Port>, UUID::Hasher> port_map_;

    std::shared_ptr<Timer> profiling_timer_;
};

}  // namespace csapex
#endif  // DESIGNER_SCENE_H
