#ifndef OVERLAY_H
#define OVERLAY_H

/// SYSTEM
#include <QWidget>
#include <QPainter>

namespace vision_evaluator
{

/// FORWARD DECLARATION
class Connector;

namespace command
{
class AddConnection;
}

class Overlay : public QWidget
{
    Q_OBJECT

    friend class command::AddConnection;
    friend class DesignerIO;

private:
    typedef std::pair<Connector*, Connector*> ConnectionPair;
    typedef std::vector<ConnectionPair> ConnectionList;

public:
    Overlay(QWidget* parent = 0);

    void drawTemporaryLine(const QLine& line);
    void deleteTemporaryLine();
    void connectorRemoved(Connector* c);

public Q_SLOTS:
    void connectorRemoved(QObject* o);
    void showPublisherSignal(Connector* c);
    void tick();
    void clear();

protected:
    void drawActivity(int life, Connector* c);
    void drawConnection(const QPoint& p1, const QPoint& p2);
    void paintEvent(QPaintEvent* event);

private:
    /// PRIVATE: Use command to spawn objects (undoable)
    void addConnection(Connector* from, Connector* to);
    void removeConnection(Connector* from, Connector* to);

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
