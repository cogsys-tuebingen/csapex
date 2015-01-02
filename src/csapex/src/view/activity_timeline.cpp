/// HEADER
#include <csapex/view/activity_timeline.h>

/// COMPONENT
#include <csapex/view/widget_controller.h>
#include <csapex/model/node_worker.h>
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
    : scene_(new QGraphicsScene), time_(0)
{
    setAlignment(Qt::AlignLeft | Qt::AlignTop);

    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

    setScene(scene_);

    setFixedHeight(row_height);

    params_.resolution = 2.0;

    start_time_ = QDateTime::currentMSecsSinceEpoch();

    timer_ = new QTimer;
    timer_->setSingleShot(false);
    timer_->setInterval(25);
    timer_->start();

    QObject::connect(timer_, SIGNAL(timeout()), this, SLOT(update()));
}

ActivityTimeline::~ActivityTimeline()
{
    foreach(Row* r, rows_) {
        delete r;
        r = NULL;
    }
}

void ActivityTimeline::resizeToFit()
{
//    QRectF rect = scene_->sceneRect();
//    rect.setHeight(row_height * rows_.size() + 1);
//    scene_->setSceneRect(rect);
    setFixedHeight(scene_->sceneRect().height() + horizontalScrollBar()->height() + 4);
}

void ActivityTimeline::addNode(NodeWorkerPtr node)
{
    int width = std::max(10000l, time_ + 1000l) / params_.resolution;
    int row = rows_.size();

    Row* r = new Row(params_, scene_, row, width, node.get());
    rows_.push_back(r);
    node2row[node.get()] = r;

    resizeToFit();

    NodeWorker* worker = node.get();

    QObject::connect(worker, SIGNAL(timerStarted(NodeWorker*, int, long)), this, SLOT(updateRowStart(NodeWorker*, int, long)));
    QObject::connect(worker, SIGNAL(timerStopped(NodeWorker*, long)), this, SLOT(updateRowStop(NodeWorker*, long)));

}

void ActivityTimeline::removeNode(NodeWorkerPtr node)
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
        if(rows_[r]->node_ == node.get()) {
            found = true;
            Row* row = rows_.at(r);
            row->clear();
            delete row;

        }
    }

    if(found) {
        node2row.erase(node.get());
        rows_.pop_back();
        resizeToFit();
    }
}

void ActivityTimeline::setSelection(QList<NodeWorker *> nodes)
{
    for(std::map<NodeWorker*,Row*>::iterator it = node2row.begin(); it != node2row.end(); ++it) {
        it->second->selected = false;
    }
    foreach(NodeWorker* node, nodes) {
        node2row.at(node)->selected = true;
    }
    refresh();
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
    row->activities_.push_back(new Activity(&params_, row, time_, static_cast<NodeWorker::ActivityType>(type)));
    row->active_activity_ = row->activities_.back();
    scene_->addItem(row->active_activity_->rect);
}

void ActivityTimeline::updateRowStop(NodeWorker* node, long stamp)
{
    Row* row = node2row.at(node);

    updateTime(stamp);
    row->active_activity_->stop(time_);
    row->active_activity_ = NULL;
}

void ActivityTimeline::updateTime(long stamp)
{
    time_ = (stamp - start_time_);
}

void ActivityTimeline::update()
{
    updateTime(QDateTime::currentMSecsSinceEpoch());

    int i = 0;
    int width = std::max(10000l, time_ + 1000l) / params_.resolution;

    for(std::map<NodeWorker*, Row*>::iterator it = node2row.begin(); it != node2row.end(); ++it) {
        Row* row = it->second;
        row->updateLines(width);

        if(row->active_activity_) {
            row->active_activity_->step(time_);
        }

        ++i;
    }
}

void ActivityTimeline::refresh()
{
    int i = 0;
    int width = std::max(10000l, time_ + 1000l) / params_.resolution;
    for(std::map<NodeWorker*, Row*>::iterator it = node2row.begin(); it != node2row.end(); ++it) {
        Row* row = it->second;
        row->updateLines(width);
        row->refresh();
        ++i;
    }
}


ActivityTimeline::Row::Row(Parameters& params, QGraphicsScene* scene, int row, int width, NodeWorker* worker)
    : params_(params), node_(worker), row(row), active_activity_(NULL), selected(false)
{
    int top = row * row_height;
    int bottom = (row+1) * row_height;

    QLineF line_top(0, top, width, top);
    line_top_item = scene->addLine(line_top);

    QLineF line_bottom(0, bottom, width, bottom);
    line_bottom_item = scene->addLine(line_bottom);

    updateLines(width);
}

ActivityTimeline::Row::~Row()
{
    clear();
    delete line_bottom_item;
    delete line_top_item;
}

void ActivityTimeline::Row::updateLines(int w)
{
    width = w;

    int top = row * row_height;
    int bottom = (row+1) * row_height;

    line_top_item->setLine(QLineF(0, top, width, top));
    line_bottom_item->setLine(QLineF(0, bottom, width, bottom));
}

void ActivityTimeline::Row::refresh()
{
    updateLines(width);

    for(std::size_t j = 0, total = activities_.size(); j < total; ++j) {
        activities_[j]->update();
    }
}

void ActivityTimeline::Row::clear()
{
    foreach(Activity* a, activities_) {
        delete a;
        a = NULL;
    }
    activities_.clear();
    active_activity_ = NULL;
}

ActivityTimeline::Activity::Activity(Parameters* params, Row *row, int start_time, NodeWorker::ActivityType type)
    : params_(params), row(row), type_(type), start_(start_time), stop_(start_time + 10)
{
    rect = new QGraphicsRectItem;

    update();
}

ActivityTimeline::Activity::~Activity()
{
    clear();
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
        color = QColor::fromRgbF(0.5, 0.5, 0.5, 1.0);
        break;
    }
    if(!row->selected) {
        color = color.lighter();
    }

    rect->setBrush(QBrush(color/*, Qt::Dense4Pattern*/));
    rect->setPen(Qt::NoPen);

    int bottom = row->line_top_item->line().p1().y();
    rect->setRect(start_ / params_->resolution, bottom, (stop_ - start_)  / params_->resolution, row_height);
}

void ActivityTimeline::Activity::clear()
{
    delete rect;
    rect = NULL;
}
