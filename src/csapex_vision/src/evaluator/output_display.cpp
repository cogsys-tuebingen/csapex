/// HEADER
#include "output_display.h"

/// PROJECT
#include <csapex/box.h>
#include <csapex/connector_in.h>
#include <utils/LibUtil/QtCvImageConverter.h>
#include <csapex_vision/messages_default.hpp>

/// SYSTEM
#include <QPainter>
#include <QGraphicsSceneEvent>
#include <QGraphicsPixmapItem>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::OutputDisplay, csapex::BoxedObject)


using namespace csapex;
using namespace connection_types;

OutputDisplay::OutputDisplay()
    : input_(NULL), pixmap_(NULL), view_(new QGraphicsView), empty(400, 400, QImage::Format_RGB16), painter(&empty), down_(false)
{
    painter.setPen(QPen(Qt::red));
    painter.fillRect(QRect(0, 0, empty.width(), empty.height()), Qt::white);
    painter.drawRect(QRect(0, 0, empty.width()-1, empty.height()-1));

    addTag(Tag::get("General"));
    addTag(Tag::get("Vision"));

    setIcon(QIcon(":/picture.png"));
}

OutputDisplay::~OutputDisplay()
{
}

bool OutputDisplay::eventFilter(QObject *o, QEvent *e)
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

            state.width = view_->width() + delta.x();
            state.height = view_->height() + delta.y();

            view_->setFixedSize(QSize(state.width, state.height));
        }
        e->accept();
        return true;

    default:
        break;
    }

    return false;
}

void OutputDisplay::fill(QBoxLayout* layout)
{
    if(input_ == NULL) {
        input_ = new ConnectorIn(box_, 0);
        input_->setLabel("Image");
        box_->addInput(input_);

        view_->setFixedSize(QSize(state.width, state.height));
        view_->setMouseTracking(true);
        QGraphicsScene* scene = view_->scene();
        if(scene == NULL) {
            scene = new QGraphicsScene();
            view_->setScene(scene);
            scene->installEventFilter(this);
        }

        layout->addWidget(view_);

        disable();

        connect(this, SIGNAL(displayRequest(QSharedPointer<QImage>)), this, SLOT(display(QSharedPointer<QImage>)));
    }
}

Memento::Ptr OutputDisplay::getState() const
{
    boost::shared_ptr<State> memento(new State);
    *memento = state;

    return memento;
}

void OutputDisplay::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state = *m;

    view_->setFixedSize(QSize(state.width, state.height));
}

void OutputDisplay::enable()
{
}

void OutputDisplay::disable()
{
    if(pixmap_ != NULL) {
        pixmap_->setPixmap(QPixmap::fromImage(empty));
        view_->fitInView(view_->scene()->sceneRect(), Qt::KeepAspectRatio);
        view_->scene()->update();
    }
}

void OutputDisplay::connectorChanged()
{
    if(input_->isConnected()) {
        enable();
    } else {
        disable();
    }
}

void OutputDisplay::display(QSharedPointer<QImage> img)
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

    view_->fitInView(view_->scene()->sceneRect(), Qt::KeepAspectRatio);
    view_->scene()->update();
}

void OutputDisplay::messageArrived(ConnectorIn* source)
{
    ConnectionType::Ptr msg = source->getMessage();
    CvMatMessage::Ptr mat_msg = boost::dynamic_pointer_cast<CvMatMessage> (msg);

    if(mat_msg.get() && !mat_msg->value.empty()) {
        QSharedPointer<QImage> img = QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(mat_msg->value);
        enable();

        Q_EMIT displayRequest(img);
    }
}
