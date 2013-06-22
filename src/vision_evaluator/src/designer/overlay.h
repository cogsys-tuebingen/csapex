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

    void connectorRemoved(Connector* c);

public Q_SLOTS:
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
    void drawConnection(const QPoint& p1, const QPoint& p2);
    void paintEvent(QPaintEvent* event);

protected:
    QLine temp_line;
    QPainter* painter;
    ConnectionList connections;

    QTimer* repainter;

    int activity_marker_max_lifetime_;
    int activity_marker_min_width_;
    int activity_marker_max_width_;
    int activity_marker_min_opacity_;
    int activity_marker_max_opacity_;

    std::vector<std::pair<int, Connector*> > publisher_signals_;
};

}
#endif // OVERLAY_H
