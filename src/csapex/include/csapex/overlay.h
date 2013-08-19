#ifndef OVERLAY_H
#define OVERLAY_H

/// SYSTEM
#include <QWidget>
#include <QPainter>

namespace csapex
{

/// FORWARD DECLARATION
class Connector;
class ConnectorOut;
class ConnectorIn;
class Connection;

class Overlay : public QWidget
{
    Q_OBJECT

public:
    Overlay(QWidget* parent = 0);

public Q_SLOTS:
    void addTemporaryConnection(Connector* from, Connector* to);
    void addTemporaryConnection(Connector *from, const QPoint& end);
    void deleteTemporaryConnections();
    void deleteTemporaryConnectionsAndRepaint();

    void tick();

    void invalidateSchema();
    void refresh();

public:
    bool mouseMoveEventHandler(QMouseEvent * e);
    bool mousePressEventHandler(QMouseEvent * e);
    bool mouseReleaseEventHandler(QMouseEvent * e);

    bool keyPressEventHandler(QKeyEvent* e);
    bool keyReleaseEventHandler(QKeyEvent* e);

    void setSelectionRectangle(const QPoint& a, const QPoint& b);

protected:
    void drawActivity(int life, Connector* c);
    void drawConnector(Connector* c);
    void drawConnection(Connection& connection);
    void drawConnection(const QPoint& from, const QPoint& to, int id, bool selected, bool highlighted, bool error = false);

    void paintEvent(QPaintEvent* event);
    void resizeEvent(QResizeEvent * event);

protected:
    struct TempConnection {
        Connector* from;
        QPoint to;
    };

    std::vector<TempConnection> temp_;

    QPainter* painter;
    QPainter* schematics_painter;
    QImage schematics;

    QTimer* repainter;

    int activity_marker_min_width_;
    int activity_marker_max_width_;
    int activity_marker_min_opacity_;
    int activity_marker_max_opacity_;

    QColor color_connection;

    QColor color_connected;
    QColor color_disconnected;

    QColor color_in_connected;
    QColor color_in_disconnected;

    QColor color_out_connected;
    QColor color_out_disconnected;


    int highlight_connection_id_;
    bool schema_dirty_;

    int connector_radius_;

    QPoint selection_a;
    QPoint selection_b;
};

}
#endif // OVERLAY_H
