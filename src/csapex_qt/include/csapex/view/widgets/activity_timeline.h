#ifndef ACTIVITY_TIMELINE_H
#define ACTIVITY_TIMELINE_H

/// COMPONENT
#include <csapex/model/activity_type.h>
#include <csapex/model/model_fwd.h>
#include <csapex_qt_export.h>
#include <csapex/utility/slim_signal.hpp>

/// SYSTEM
#include <QGraphicsView>

namespace csapex
{

class ActivityTimelineItem;
class Interval;

class CSAPEX_QT_EXPORT ActivityTimeline : public QGraphicsView
{
    Q_OBJECT

public:
    ActivityTimeline();
    ~ActivityTimeline();

    virtual void drawBackground(QPainter *painter, const QRectF &rect);
    virtual void drawForeground(QPainter *painter, const QRectF &rect);

public Q_SLOTS:
    void addNode(NodeFacade* node);
    void removeNode(NodeFacade* node);

    void setSelection(QList<NodeFacade*>);

    void updateRecording();
    void setRecording(bool recording);

    void reset();

    void refresh();
    void update();
    void updateTime();
    void updateTime(long stamp);
    void updateRowStart(NodeFacade* worker, ActivityType type, std::shared_ptr<const Interval> profile);
    void updateRowStop(NodeFacade* worker, std::shared_ptr<const Interval> profile);

    void wheelEvent(QWheelEvent *we);

    void resizeToFit();

    void addItem(QGraphicsItem* item);


Q_SIGNALS:
    void recordingChanged(bool);

    void updateRowStartRequest(NodeFacade* worker, ActivityType type, std::shared_ptr<const Interval> profile);
    void updateRowStopRequest(NodeFacade* worker, std::shared_ptr<const Interval> profile);

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
        Activity(Parameters* params, Row* row, int start_time, ActivityType type, std::shared_ptr<Interval const> interval);
        ~Activity();

        void step(int time);
        void stop(int stop_time);
        void update();

        Parameters* params_;
        Row* row;

        ActivityType type_;
        std::shared_ptr<Interval const> interval_;

        int start_;
        int stop_;

        ActivityTimelineItem* rect;
    };

    struct Row
    {
        Row(Parameters& params, QGraphicsScene* scene, int row, NodeFacade* worker);
        ~Row();

        void refresh();
        void clear();

    public:
        Parameters& params_;
        NodeFacade* node_;

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

    bool recording_;

    Parameters params_;

    std::vector<Row*> rows_;
    std::map<NodeFacade*, Row*> node2row;
    std::map<NodeFacade*, std::vector<csapex::slim_signal::ScopedConnection>> node2connections_;
};

}

#endif // ACTIVITY_TIMELINE_H

