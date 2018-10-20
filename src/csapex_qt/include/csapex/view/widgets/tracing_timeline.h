#ifndef TRACING_TIMELINE_H
#define TRACING_TIMELINE_H

/// COMPONENT
#include <csapex/model/tracing_type.h>
#include <csapex/model/model_fwd.h>
#include <csapex_qt_export.h>
#include <csapex/utility/slim_signal.hpp>

/// SYSTEM
#include <QGraphicsView>

namespace csapex
{
class TracingTimelineItem;
class Interval;

class CSAPEX_QT_EXPORT TracingTimeline : public QGraphicsView
{
    Q_OBJECT

public:
    TracingTimeline();
    ~TracingTimeline();

    virtual void drawBackground(QPainter* painter, const QRectF& rect);
    virtual void drawForeground(QPainter* painter, const QRectF& rect);

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
    void updateRowStart(NodeFacade* worker, TracingType type, std::shared_ptr<const Interval> profile);
    void updateRowStop(NodeFacade* worker, std::shared_ptr<const Interval> profile);

    void wheelEvent(QWheelEvent* we);

    void resizeToFit();

    void addItem(QGraphicsItem* item);

Q_SIGNALS:
    void recordingChanged(bool);

    void updateRowStartRequest(NodeFacade* worker, TracingType type, std::shared_ptr<const Interval> profile);
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

    struct Trace
    {
        Trace(Parameters* params, Row* row, int start_time, TracingType type, std::shared_ptr<Interval const> interval);
        ~Trace();

        void step(int time);
        void stop(int stop_time);
        void update();

        Parameters* params_;
        Row* row;

        TracingType type_;
        std::shared_ptr<Interval const> interval_;

        int start_;
        int stop_;

        TracingTimelineItem* rect;
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

        std::vector<Trace*> traces_;
        Trace* active_trace_;

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

}  // namespace csapex

#endif  // TRACING_TIMELINE_H
