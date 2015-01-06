#ifndef ACTIVITY_TIMELINE_H
#define ACTIVITY_TIMELINE_H

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/node_worker.h>

/// SYSTEM
#include <QGraphicsView>

namespace csapex
{

class ActivityTimeline : public QGraphicsView
{
    Q_OBJECT

public:
    ActivityTimeline();
    ~ActivityTimeline();

    virtual void drawBackground(QPainter *painter, const QRectF &rect);
    virtual void drawForeground(QPainter *painter, const QRectF &rect);

public Q_SLOTS:
    void addNode(NodeWorker* node);
    void removeNode(NodeWorker* node);

    void setSelection(QList<NodeWorker*>);

    void updateScrolling();
    void setScrolling(bool scrolling);

    void reset();

    void refresh();
    void update();
    void updateTime();
    void updateTime(long stamp);
    void updateRowStart(NodeWorker* worker, int type, long stamp);
    void updateRowStop(NodeWorker* worker, long stamp);

    void wheelEvent(QWheelEvent *we);

    void resizeToFit();


Q_SIGNALS:
    void scrollingChanged(bool);

private:
    void startTimer();
    void stopTimer();

private:
    struct Row;

    struct Parameters
    {
        double resolution;

        long start_time_stamp;

        long start_time;
        long time;
    };

    struct Activity
    {
        Activity(Parameters* params, Row* row, int start_time, NodeWorker::ActivityType type);
        ~Activity();

        void step(int time);
        void stop(int stop_time);
        void update();
        void clear();

        Parameters* params_;
        Row* row;

        NodeWorker::ActivityType type_;
        int start_;
        int stop_;

        QGraphicsRectItem* rect;
    };

    struct Row
    {
        Row(Parameters& params, QGraphicsScene* scene, int row, NodeWorker* worker);
        ~Row();

        void refresh();
        void clear();

    public:
        Parameters& params_;
        NodeWorker* node_;

        int row;
        int top;
        int bottom;

        std::vector<Activity*> activities_;
        Activity* active_activity_;

        bool selected;
    };

private:
    QGraphicsScene* scene_;
    QTimer* timer_;

    bool scrolling_;

    Parameters params_;

    std::vector<Row*> rows_;
    std::map<NodeWorker*, Row*> node2row;
};

}

#endif // ACTIVITY_TIMELINE_H

