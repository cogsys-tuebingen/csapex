/// HEADER
#include "output_display.h"

/// COMPONENT
#include "registration.hpp"
#include "messages_default.hpp"

/// PROJECT
#include <designer/box.h>
#include <designer/connector_in.h>
#include <utils/LibUtil/QtCvImageConverter.h>

/// SYSTEM
#include <QPainter>

STATIC_INIT(OutputDisplay, generic, {
                SelectorProxy::ProxyConstructor c; \
                c.setName("Output Display"); \
                c.setConstructor(boost::lambda::bind(boost::lambda::new_ptr<SelectorProxyImp<OutputDisplay> >(), \
                boost::lambda::_1, (QWidget*) NULL)); \
                SelectorProxy::registerProxy(c); \
            });

using namespace vision_evaluator;

OutputDisplay::OutputDisplay()
    : input_(NULL), view_(new QGraphicsView), empty(400, 400, QImage::Format_RGB16), painter(&empty)
{
    painter.setPen(QPen(Qt::red));
    painter.fillRect(QRect(0, 0, empty.width(), empty.height()), Qt::white);
    painter.drawRect(QRect(0, 0, empty.width()-1, empty.height()-1));
}

OutputDisplay::~OutputDisplay()
{
}

void OutputDisplay::fill(QBoxLayout* layout)
{
    if(input_ == NULL) {
        input_ = new ConnectorIn(box_, 0);
        box_->addInput(input_);

        view_->setFixedSize(QSize(300, 300));
        QGraphicsScene* scene = view_->scene();
        if(scene == NULL) {
            scene = new QGraphicsScene();
            view_->setScene(scene);
        }

        display_is_empty = false;
        layout->addWidget(view_);

        disable();

        connect(input_, SIGNAL(messageArrived(ConnectorIn*)), this, SLOT(messageArrived(ConnectorIn*)));
    }
}

void OutputDisplay::enable()
{
}

void OutputDisplay::disable()
{
    if(!display_is_empty) {
        pixmap_ = QPixmap::fromImage(empty);
        if(view_->scene()) {
            delete view_->scene();
        }
        view_->setScene(new QGraphicsScene());
        view_->scene()->addPixmap(pixmap_);
        view_->fitInView(view_->scene()->sceneRect(), Qt::KeepAspectRatio);
        view_->scene()->update();

        display_is_empty = true;
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

void OutputDisplay::messageArrived(ConnectorIn* source)
{
    if(!isEnabled()) {
        disable();
        return;
    }

    ConnectionType::Ptr msg = source->getMessage();
    CvMatMessage::Ptr mat_msg = boost::dynamic_pointer_cast<CvMatMessage> (msg);

    if(mat_msg.get() && !mat_msg->value.empty()) {
        QSharedPointer<QImage> img = QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(mat_msg->value);
        enable();

        if(display_is_empty || img->rect() != pixmap_.rect()) {
            if(view_->scene()) {
                delete view_->scene();
            }
            view_->setScene(new QGraphicsScene());
            pixmap_ = QPixmap::fromImage(*img);
            view_->scene()->addPixmap(pixmap_);
            display_is_empty = false;

        } else {
            pixmap_.convertFromImage(*img);
        }
        view_->fitInView(view_->scene()->sceneRect(), Qt::KeepAspectRatio);
        view_->scene()->update();
    }
}
