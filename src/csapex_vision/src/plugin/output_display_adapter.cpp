/// HEADER
#include "output_display_adapter.h"

/// PROJECT
#include <csapex/model/connector_in.h>
#include <csapex/utility/register_node_adapter.h>

/// SYSTEM
#include <QPainter>
#include <QGraphicsSceneEvent>
#include <QGraphicsPixmapItem>
#include <QCheckBox>
#include <QPushButton>

using namespace csapex;

CSAPEX_REGISTER_NODE_ADAPTER(csapex::OutputDisplayAdapter, csapex::OutputDisplay)


OutputDisplayAdapter::OutputDisplayAdapter(OutputDisplay *node)
    : NodeAdapter(node), wrapped_(node), pixmap_(NULL), view_(new QGraphicsView), empty(32, 32, QImage::Format_RGB16), painter(&empty), down_(false)
{
    painter.setPen(QPen(Qt::red));
    painter.fillRect(QRect(0, 0, empty.width(), empty.height()), Qt::white);
    painter.drawRect(QRect(0, 0, empty.width()-1, empty.height()-1));

    // translate to UI thread via Qt signal
    node->display_request.connect(boost::bind(&OutputDisplayAdapter::displayRequest, this, _1));
}


bool OutputDisplayAdapter::eventFilter(QObject *o, QEvent *e)
{
    QGraphicsSceneMouseEvent* me = dynamic_cast<QGraphicsSceneMouseEvent*> (e);

    switch(e->type()) {
    case QEvent::GraphicsSceneMousePress:
        down_ = true;
        last_pos_ = me->screenPos();
        e->accept();
        return true;
    case QEvent::GraphicsSceneMouseRelease:
        down_ = false;
        e->accept();
        return true;
    case QEvent::GraphicsSceneMouseMove:
        if(down_) {
            QPoint delta = me->screenPos() - last_pos_;

            last_pos_ = me->screenPos();

            state.width = std::max(32, view_->width() + delta.x());
            state.height = std::max(32, view_->height() + delta.y());

            view_->setFixedSize(QSize(state.width, state.height));
        }
        e->accept();
        return true;

    default:
        break;
    }

    return false;
}

void OutputDisplayAdapter::setupUi(QBoxLayout* layout)
{
    view_->setFixedSize(QSize(state.width, state.height));
    view_->setMouseTracking(true);
    view_->setAcceptDrops(false);
    QGraphicsScene* scene = view_->scene();
    if(scene == NULL) {
        scene = new QGraphicsScene();
        view_->setScene(scene);
        scene->installEventFilter(this);
    }

    layout->addWidget(view_);

    QHBoxLayout* sub = new QHBoxLayout;

    QCheckBox* cb = new QCheckBox("async");
    cb->setChecked(wrapped_->input_->isAsync());
    sub->addWidget(cb, 0,  Qt::AlignLeft);
    QObject::connect(cb, SIGNAL(stateChanged(int)), this, SLOT(setAsync(int)));

    QPushButton* fit = new QPushButton("fit size");
    sub->addWidget(fit, 0,  Qt::AlignLeft);
    QObject::connect(fit, SIGNAL(clicked()), this, SLOT(fitInView()));

    sub->addSpacerItem(new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Minimum));

    layout->addLayout(sub);

//    disable();

    connect(this, SIGNAL(displayRequest(QSharedPointer<QImage>)), this, SLOT(display(QSharedPointer<QImage>)));
}

void OutputDisplayAdapter::fitInView()
{
    if(last_size_.isNull()) {
        return;
    }

    state.width = last_size_.width();
    state.height = last_size_.height();
    view_->setFixedSize(QSize(state.width, state.height));
}

void OutputDisplayAdapter::setAsync(int a)
{
    wrapped_->input_->setAsync(a);
}

Memento::Ptr OutputDisplayAdapter::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void OutputDisplayAdapter::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state = *m;

    view_->setFixedSize(QSize(state.width, state.height));
}

void OutputDisplayAdapter::display(QSharedPointer<QImage> img)
{
    if(pixmap_ == NULL) {
        if(view_->scene()) {
            delete view_->scene();
        }
        view_->setScene(new QGraphicsScene());
        view_->scene()->installEventFilter(this);

        pixmap_ = view_->scene()->addPixmap(QPixmap::fromImage(*img));

    } else {
        pixmap_->setPixmap(QPixmap::fromImage(*img));
    }

    last_size_ = img->size();

    view_->scene()->setSceneRect(img->rect());
    view_->fitInView(view_->scene()->sceneRect(), Qt::KeepAspectRatio);
    view_->scene()->update();
}
