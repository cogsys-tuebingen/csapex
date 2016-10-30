/// HEADER
#include <csapex/view/widgets/activity_timeline.h>

/// COMPONENT
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/profiling/interval.h>
#include <csapex/view/widgets/activity_timeline_item.h>

/// SYSTEM
#include <QPainter>
#include <QGraphicsLineItem>
#include <QSignalMapper>
#include <QTimer>
#include <QDateTime>
#include <QApplication>
#include <QWheelEvent>
#include <QScrollBar>

using namespace csapex;

namespace {
static const int row_height = 30;
}

ActivityTimeline::ActivityTimeline()
    : scene_(new QGraphicsScene), recording_(true)
{
    setAlignment(Qt::AlignLeft | Qt::AlignTop);

    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    setCacheMode(QGraphicsView::CacheNone);
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

    setAutoFillBackground(true);

    setScene(scene_);

    setFixedHeight(row_height);

    setRecording(false);

    params_.resolution = 0.5; /*ms*/
    params_.time = 0;
    params_.start_time_stamp = QDateTime::currentMSecsSinceEpoch();

    timer_ = nullptr;

    setVisible(false);

    QObject::connect(horizontalScrollBar(), &QScrollBar::sliderMoved,
                     this, &ActivityTimeline::updateRecording);
    QObject::connect(this, &ActivityTimeline::updateRowStartRequest,
                     this, &ActivityTimeline::updateRowStart);
    QObject::connect(this, &ActivityTimeline::updateRowStopRequest,
                     this, &ActivityTimeline::updateRowStop);
}

ActivityTimeline::~ActivityTimeline()
{
    for(Row* r : rows_) {
        delete r;
        r = nullptr;
    }
}

void ActivityTimeline::resizeToFit()
{
    // adjust resolution to fit the width
    double duration = std::max<double>(100, params_.time - params_.start_time);

    params_.resolution = duration / width();

    QRectF rect = scene_->sceneRect();

    rect.setHeight(row_height * rows_.size() + 1);
    rect.setWidth((params_.time - params_.start_time) / params_.resolution);
    scene_->setSceneRect(rect);

    refresh();

    setFixedHeight(scene_->sceneRect().height() + horizontalScrollBar()->height() + 5);
}

void ActivityTimeline::addItem(QGraphicsItem *item)
{
    scene_->addItem(item);
}


void ActivityTimeline::drawBackground(QPainter *painter, const QRectF &rect)
{
    QGraphicsView::drawBackground(painter, rect);

//    painter->fillRect(rect, Qt::white);

    double left = rect.x();
    double right = rect.x() + rect.width();
    double top = rect.y();
    double bottom = rect.y() + rect.height();

    double width_of_a_millisecond = 1.0 / params_.resolution;

    // separate lanes
    for(std::size_t r = 0; r < rows_.size() + 1; ++r) {
        double y = r * row_height;
        painter->drawLine(left,y, right,y);
    }

    // second tick marks
    painter->setPen(QPen(QBrush(QColor::fromRgbF(0.0, 0.0, 0.0, 0.5)), 10));

    double width_of_a_second = 1000.0 * width_of_a_millisecond;
    for(int second = 0, seconds = (params_.time - params_.start_time) / 1000.0; second <= seconds; ++second) {
        double tick = second * width_of_a_second;

        painter->drawLine(tick,top, tick,bottom);

        QString text = QString::number(tick);
        painter->drawText(QRectF(tick-10, 10, 20, 20), text);
    }

    // centisecond tick marks
    double width_of_a_centisecond = 10.0 * width_of_a_millisecond;
    if(width_of_a_centisecond > 10.0) {
        painter->setPen(QPen(QBrush(QColor::fromRgbF(0.0, 0.0, 0.0, 0.5)), 3));

        for(int centisecond = 0, centiseconds = (params_.time - params_.start_time) / 10.0; centisecond <= centiseconds; ++centisecond) {
            double tick = centisecond * width_of_a_centisecond;

            painter->drawLine(tick,top, tick,bottom);
        }
    }

    // millisecond tick marks
    if(width_of_a_millisecond > 10.0) {
        painter->setPen(QPen(QBrush(QColor::fromRgbF(0.0, 0.0, 0.0, 0.5)), 1));

        for(int millisecond = 0, milliseconds = (params_.time - params_.start_time); millisecond <= milliseconds; ++millisecond) {
            double tick = millisecond * width_of_a_millisecond;

            painter->drawLine(tick,top, tick,bottom);
        }
    }
}

void ActivityTimeline::drawForeground(QPainter *painter, const QRectF &rect)
{
    QGraphicsView::drawForeground(painter, rect);
}

void ActivityTimeline::startTimer()
{
    timer_ = new QTimer;
    timer_->setSingleShot(false);
    timer_->setInterval(25);
    timer_->start();

    QObject::connect(timer_, SIGNAL(timeout()), this, SLOT(update()));

    setVisible(true);

    reset();
}

void ActivityTimeline::stopTimer()
{
    timer_->stop();
    QObject::disconnect(timer_);
    timer_->deleteLater();
    timer_ = nullptr;

    setVisible(false);
}


void ActivityTimeline::addNode(NodeWorker* node)
{
    if(rows_.empty()) {
        startTimer();
    }

    int row = rows_.size();

    Row* r = new Row(params_, scene_, row, node);
    rows_.push_back(r);
    node2row[node] = r;

    resizeToFit();

    node2connections_[node].emplace_back(node->interval_start.connect(this, &ActivityTimeline::updateRowStartRequest));
    node2connections_[node].emplace_back(node->interval_end.connect(this, &ActivityTimeline::updateRowStopRequest));
}

void ActivityTimeline::removeNode(NodeWorker* node)
{
    bool found = false;
    for(std::size_t r = 0; r < rows_.size(); ++r) {
        if(found) {
            // deleted -> move one up
            rows_[r-1] = rows_[r];
            node2row[rows_[r-1]->node_] = rows_[r-1];
            rows_[r-1]->row = r - 1;
            rows_[r-1]->refresh();
        }
        if(rows_[r]->node_ == node) {
            found = true;
            Row* row = rows_.at(r);
            row->clear();
            delete row;

        }
    }

    if(found) {
        node2row.erase(node);
        rows_.pop_back();

        for(csapex::slim_signal::Connection& c : node2connections_[node]) {
            c.disconnect();
        }
        node2connections_.erase(node);

        resizeToFit();
    }

    if(rows_.empty()) {
        stopTimer();
    }
}

void ActivityTimeline::setSelection(QList<NodeWorker *> nodes)
{
    for(std::map<NodeWorker*,Row*>::iterator it = node2row.begin(); it != node2row.end(); ++it) {
        it->second->selected = false;
    }
    for(NodeWorker* node : nodes) {
        node2row.at(node)->selected = true;
    }
    refresh();
}

void ActivityTimeline::updateRecording()
{
}

void ActivityTimeline::setRecording(bool recording)
{
    if(recording != recording_) {
        recording_ = recording;
        Q_EMIT recordingChanged(recording);

        if(recording) {
            reset();
        }
    }
}

void ActivityTimeline::wheelEvent(QWheelEvent *we)
{
    bool ctrl = Qt::ControlModifier & QApplication::keyboardModifiers();

    if(ctrl) {
        QPointF old_scene_pos = mapToScene(we->pos());
        double time_anchor = old_scene_pos.x() * params_.resolution;

        QPointF old_center = mapToScene(viewport()->rect().center());
        double offset_x = old_center.x() - old_scene_pos.x();

        double old_res = params_.resolution;

        params_.resolution *= we->delta() < 0 ? 1.25 : 0.75;

        if(params_.resolution < 0.001) {
            params_.resolution = 0.001;
        } else if(params_.resolution > 1000.0) {
            params_.resolution = 1000.0;
        }

        if(params_.resolution == old_res) {
            return;
        }

        QRectF rect = scene_->sceneRect();
        rect.setHeight(row_height * rows_.size() + 1);
        rect.setWidth((params_.time - params_.start_time) / params_.resolution);
        scene_->setSceneRect(rect);

        double new_scene_pos_x = time_anchor / params_.resolution;
        centerOn(new_scene_pos_x + offset_x, we->pos().y());

        invalidateScene();

        refresh();

    } else {
        QGraphicsView::wheelEvent(we);
    }
}

void ActivityTimeline::updateRowStart(NodeWorker* node, int type, std::shared_ptr<const Interval> interval)
{
    if(!recording_) {
        return;
    }

    Row* row = node2row.at(node);

    updateTime(interval->getStartMs());
    row->activities_.push_back(new Activity(&params_, row, params_.time, static_cast<NodeWorker::ActivityType>(type), interval));
    row->active_activity_ = row->activities_.back();

    addItem(row->active_activity_->rect);
}

void ActivityTimeline::updateRowStop(NodeWorker* node, std::shared_ptr<const Interval> interval)
{
    if(!recording_) {
        return;
    }

    try {
        Row* row = node2row.at(node);
        if(!row->active_activity_) {
            return;
        }

        updateTime(interval->getEndMs());
        row->active_activity_->stop(params_.time);
        row->active_activity_ = nullptr;

    } catch(const std::out_of_range& e) {
        // ignore
    }
}

void ActivityTimeline::updateTime()
{
    updateTime(QDateTime::currentMSecsSinceEpoch());
}

void ActivityTimeline::updateTime(long stamp)
{
    if(recording_) {
        params_.time = (stamp - params_.start_time_stamp);
    }
}

void ActivityTimeline::update()
{
    updateTime();

    int i = 0;

    for(std::map<NodeWorker*, Row*>::iterator it = node2row.begin(); it != node2row.end(); ++it) {
        Row* row = it->second;

        if(row->active_activity_) {
            row->active_activity_->step(params_.time);
        }

        ++i;
    }


    if(recording_) {
        resizeToFit();
        //QScrollBar* bar = horizontalScrollBar();
        //bar->setValue(bar->maximum());
    }
}

void ActivityTimeline::reset()
{
    scene()->clear();

    updateTime();

    params_.start_time = params_.time;
    refresh();
}

void ActivityTimeline::refresh()
{
    int i = 0;
    for(std::map<NodeWorker*, Row*>::iterator it = node2row.begin(); it != node2row.end(); ++it) {
        Row* row = it->second;
        row->refresh();
        ++i;
    }
}


ActivityTimeline::Row::Row(Parameters& params, QGraphicsScene* /*scene*/, int row, NodeWorker* worker)
    : params_(params), node_(worker), row(row), active_activity_(nullptr), selected(false)
{
    top = row * row_height;
    bottom = (row+1) * row_height;
}

ActivityTimeline::Row::~Row()
{
    clear();
}


void ActivityTimeline::Row::refresh()
{
    for(std::size_t j = 0; j < activities_.size(); ) {
        Activity* activity = activities_[j];
        if(activity->stop_ < params_.start_time) {
            activities_.erase(activities_.begin() + j);
            if(active_activity_ == activity) {
                active_activity_ = nullptr;
            }
            delete activity;
        } else {
            activity->update();
            ++j;
        }
    }
}

void ActivityTimeline::Row::clear()
{
    for(Activity* a : activities_) {
        delete a;
        a = nullptr;
    }
    activities_.clear();
    active_activity_ = nullptr;
}

ActivityTimeline::Activity::Activity(Parameters* params, Row *row, int start_time, NodeWorker::ActivityType type, std::shared_ptr<const Interval> interval)
    : params_(params), row(row), type_(type), interval_(interval), start_(start_time), stop_(start_time + 10)
{
    rect = new ActivityTimelineItem(interval);

    update();
}

ActivityTimeline::Activity::~Activity()
{
}


void ActivityTimeline::Activity::stop(int stop_time)
{
    stop_ = stop_time;
    update();
}

void ActivityTimeline::Activity::step(int time)
{
    stop_ = time;
    update();
}

void ActivityTimeline::Activity::update()
{
    QColor color;

    switch(type_) {
    case NodeWorker::PROCESS:
        color = QColor::fromRgbF(1.0, 0.15, 0.15, 1.0);
        break;
    case NodeWorker::TICK:
        color = QColor::fromRgbF(0.15, 1.0, 0.15, 1.0);
        break;
    case NodeWorker::SLOT:
        color = QColor::fromRgbF(0.15, 0.15, 1.0, 1.0);
        break;
    case NodeWorker::OTHER:
        color = QColor::fromRgbF(0.15, 0.5, 0.5, 1.0);
        break;
    }
    if(!row->selected) {
        color = color.lighter();
    }

    QPen pen(QColor(20, 20, 20));
    if(interval_->isActive()){
        rect->setBrush(QBrush(color/*, Qt::Dense4Pattern*/));
        pen.setWidth(3);
    } else {

        rect->setBrush(QBrush(color, Qt::Dense4Pattern));
        pen.setWidth(1);
    }
    rect->setPen(pen);

    int bottom = row->top;

    double x = std::max(0.0, (start_ - params_->start_time) / params_->resolution);
    int width = (stop_ - start_)  / params_->resolution;
    rect->setRect(x, bottom, std::max(2, width), row_height);

    rect->refresh();
}
/// MOC
#include "../../../include/csapex/view/widgets/moc_activity_timeline.cpp"
