#ifndef OVERLAY_H
#define OVERLAY_H

/// SYSTEM
#include <QWidget>
#include <QPainter>

namespace vision_evaluator
{

/// FORWARD DECLARATION
class Connector;
class ConnectorOut;
class ConnectorIn;

namespace command
{
class AddConnection;
}

class Overlay : public QWidget
{
    Q_OBJECT

private:
    typedef std::pair<ConnectorOut*, ConnectorIn*> ConnectionPair;
    typedef std::vector<ConnectionPair> ConnectionList;

public:
    Overlay(QWidget* parent = 0);

public Q_SLOTS:
    void connectorRemoved(Connector* c);
    void connectorAdded(Connector* c);
    void connectorAdded(QObject* c);
    void connectorRemoved(QObject* o);
    void showPublisherSignal(Connector* c);
    void showPublisherSignal(ConnectorIn* c);
    void showPublisherSignal(ConnectorOut* c);

    void drawConnectionPreview(Connector* from, Connector* to);
    void drawTemporaryConnection(Connector *from, const QPoint& end);
    void deleteTemporaryConnection();
    void addConnection(ConnectorOut* from, ConnectorIn* to);
    void removeConnection(ConnectorOut* from, ConnectorIn* to);
    void tick();
    void clear();

protected:
    void drawActivity(int life, Connector* c);
    void clearActivity(Connector* c);
    void drawConnector(Connector* c);
    void drawConnection(const QPoint& p1, const QPoint& p2);
    void paintEvent(QPaintEvent* event);

protected:
    QLine temp_line;
    QPainter* painter;

    std::vector<Connector*> connectors_;
    ConnectionList connections;

    QTimer* repainter;

    int activity_marker_max_lifetime_;
    int activity_marker_min_width_;
    int activity_marker_max_width_;
    int activity_marker_min_opacity_;
    int activity_marker_max_opacity_;

    QColor color_connection;

    QColor color_in_connected;
    QColor color_in_disconnected;

    QColor color_out_connected;
    QColor color_out_disconnected;

    std::vector<std::pair<int, Connector*> > publisher_signals_;
};

}
#endif // OVERLAY_H
