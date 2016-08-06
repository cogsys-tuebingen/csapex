/// HEADER
#include <csapex/view/widgets/activity_timeline.h>

/// COMPONENT
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>

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
    : scene_(new QGraphicsScene), scrolling_(true)
{
    setAlignment(Qt::AlignLeft | Qt::AlignTop);

    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

    setScene(scene_);

    setFixedHeight(row_height);

    setScrolling(false);

    params_.resolution = 0.5; /*ms*/
    params_.time = 0;
    params_.start_time_stamp = QDateTime::currentMSecsSinceEpoch();

    timer_ = nullptr;

    setVisible(false);

    QObject::connect(horizontalScrollBar(), &QScrollBar::sliderMoved,
                     this, &ActivityTimeline::updateScrolling);
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
    QRectF rect = scene_->sceneRect();
    rect.setHeight(row_height * rows_.size() + 1);
    rect.setWidth((params_.time - params_.start_time) / params_.resolution);
    scene_->setSceneRect(rect);

    setFixedHeight(scene_->sceneRect().height() + horizontalScrollBar()->height() + 5);
}

void ActivityTimeline::addItem(QGraphicsItem *item)
{
    scene_->addItem(item);
}


void ActivityTimeline::drawBackground(QPainter *painter, const QRectF &rect)
{
    QGraphicsView::drawBackground(painter, rect);
}

void ActivityTimeline::drawForeground(QPainter *painter, const QRectF &rect)
{
    QGraphicsView::drawForeground(painter, rect);

    double left = rect.x();
    double right = rect.x() + rect.width();
    double top = rect.y();
    double bottom = rect.y() + rect.height() / 2;

    for(std::size_t r = 0; r < rows_.size() + 1; ++r) {
        double y = r * row_height;
        painter->drawLine(left,y, right,y);
    }

    double scale = 1.0 / transform().m11();
    painter->setPen(QPen(QBrush(QColor::fromRgbF(0.0, 0.0, 0.0, 0.5)), scale));

    double delta = 1e3 / params_.resolution;

    for(double x = std::floor(left); x < std::ceil(right); x += delta) {
        painter->drawLine(x,top, x,bottom);

//        QString text = QString::number(x);
//        painter->drawText(QRectF(x-10, 10, 20, 20), text);
    }
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

    auto cstart = node->timerStarted.connect([this](NodeWorker* worker, int type, long stamp) {
            updateRowStartRequest(worker, type, stamp);
    });
    node2connections_[node].push_back(cstart);

    auto cstop = node->timerStopped.connect([this](NodeWorker* worker, long stamp) {
            updateRowStopRequest(worker, stamp);
    });
    node2connections_[node].push_back(cstop);
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

void ActivityTimeline::updateScrolling()
{
    if(scrolling_) {
        QScrollBar* bar = horizontalScrollBar();
        if(bar->value() < bar->maximum() - 10) {
            setScrolling(false);
        }
    }
}

void ActivityTimeline::setScrolling(bool scrolling)
{
    if(scrolling != scrolling_) {
        scrolling_ = scrolling;
        Q_EMIT scrollingChanged(scrolling);
    }
}

void ActivityTimeline::wheelEvent(QWheelEvent *we)
{
    //    bool shift = Qt::ShiftModifier & QApplication::keyboardModifiers();
    bool ctrl = Qt::ControlModifier & QApplication::keyboardModifiers();

    if(ctrl) {
        //        params_.resolution *= we->delta() < 0 ? 1.25 : 0.75;

        scale(we->delta() > 0 ? 1.25 : 0.75, 1.0);

        if(params_.resolution < 0.01) {
            params_.resolution = 0.01;
        } else if(params_.resolution > 1000.0) {
            params_.resolution = 1000.0;
        }

        refresh();

    } else {
        QGraphicsView::wheelEvent(we);
    }
}

void ActivityTimeline::updateRowStart(NodeWorker* node, int type, long stamp)
{
    Row* row = node2row.at(node);

    updateTime(stamp);
    row->activities_.push_back(new Activity(&params_, row, params_.time, static_cast<NodeWorker::ActivityType>(type)));
    row->active_activity_ = row->activities_.back();

    addItem(row->active_activity_->rect);
}

void ActivityTimeline::updateRowStop(NodeWorker* node, long stamp)
{
    try {
        Row* row = node2row.at(node);
        if(!row->active_activity_) {
            return;
        }

        updateTime(stamp);
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
    params_.time = (stamp - params_.start_time_stamp);
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

    resizeToFit();

    if(scrolling_) {
        QScrollBar* bar = horizontalScrollBar();
        bar->setValue(bar->maximum());
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

ActivityTimeline::Activity::Activity(Parameters* params, Row *row, int start_time, NodeWorker::ActivityType type)
    : params_(params), row(row), type_(type), start_(start_time), stop_(start_time + 10)
{
    rect = new QGraphicsRectItem;

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
        color = QColor::fromRgbF(1.0, 0.2, 0.2, 1.0);
        break;
    case NodeWorker::TICK:
        color = QColor::fromRgbF(0.2, 1.0, 0.2, 1.0);
        break;
    case NodeWorker::OTHER:
        color = QColor::fromRgbF(0.5, 0.5, 0.5, 1.0);
        break;
    }
    if(!row->selected) {
        color = color.lighter();
    }

    rect->setBrush(QBrush(color/*, Qt::Dense4Pattern*/));
    rect->setPen(Qt::NoPen);

    int bottom = row->top;

    double x = std::max(0.0, (start_ - params_->start_time) / params_->resolution);
    int width = (stop_ - start_)  / params_->resolution;
    rect->setRect(x, bottom, std::max(2, width), row_height);
}
/// MOC
#include "../../../include/csapex/view/widgets/moc_activity_timeline.cpp"
