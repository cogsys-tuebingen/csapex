#ifndef OVERLAY_H
#define OVERLAY_H

/// SYSTEM
#include <QWidget>
#include <QPainter>

/// FORWARD DECLARATION
class Connector;

class Overlay : public QWidget
{
    Q_OBJECT

private:
    typedef std::pair<Connector*, Connector*> ConnectionPair;
    typedef std::vector<ConnectionPair> ConnectionList;

public:
    Overlay(QWidget* parent = 0);

    void drawTemporaryLine(const QLine& line);
    void deleteTemporaryLine();
    void addConnection(Connector* from, Connector* to);
    void connectorRemoved(Connector* c);

public Q_SLOTS:
    void connectorRemoved(QObject* o);

protected:
    void drawConnection(const QPoint& p1, const QPoint& p2);
    void paintEvent(QPaintEvent* event);

protected:
    QLine temp_line;
    QPainter* painter;
    ConnectionList connections;
};

#endif // OVERLAY_H
