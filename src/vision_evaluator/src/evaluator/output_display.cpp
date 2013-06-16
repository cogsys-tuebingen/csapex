/// HEADER
#include "output_display.h"

/// COMPONENT
#include "registration.hpp"
#include "messages_default.hpp"

/// PROJECT
#include <designer/box.h>
#include <designer/connector_in.h>

#include <utils/LibUtil/QtCvImageConverter.h>

STATIC_INIT(OutputDisplay, generic, {
    SelectorProxy::ProxyConstructor c; \
    c.setName("Output Display"); \
    c.setConstructor(boost::lambda::bind(boost::lambda::new_ptr<SelectorProxyImp<OutputDisplay> >(), \
    boost::lambda::_1, (QWidget*) NULL)); \
    SelectorProxy::registerProxy(c); \
});

using namespace vision_evaluator;

OutputDisplay::OutputDisplay()
    : input_(NULL), view_(new QGraphicsView)
{
}

void OutputDisplay::fill(QBoxLayout* layout)
{
    if(input_ == NULL) {
//        makeThread();
//        this->moveToThread(private_thread_);
//        private_thread_->start();

        input_ = new ConnectorIn(box_, 0);
        box_->addInput(input_);

        view_->setFixedSize(QSize(300, 300));
        QGraphicsScene* scene = view_->scene();
        if(scene == NULL) {
            scene = new QGraphicsScene();
            view_->setScene(scene);
        }

        layout->addWidget(view_);

        connect(input_, SIGNAL(messageArrived(ConnectorIn*)), this, SLOT(messageArrived(ConnectorIn*)));
    }
}

void OutputDisplay::messageArrived(ConnectorIn* source)
{
    if(!box_->isEnabled()) {
        return;
    }

    ConnectionType::Ptr msg = source->getMessage();
    CvMatMessage::Ptr mat_msg = boost::dynamic_pointer_cast<CvMatMessage> (msg);

    if(mat_msg.get() && !mat_msg->value.empty()) {
        QSharedPointer<QImage> img = QtCvImageConverter::Converter<QImage, QSharedPointer>::mat2QImage(mat_msg->value);
        if(pixmap_.size().isEmpty()) {
            pixmap_ = QPixmap::fromImage(*img);
            view_->scene()->addPixmap(pixmap_);
            view_->fitInView(view_->scene()->sceneRect(), Qt::KeepAspectRatio);
        } else {
            pixmap_.convertFromImage(*img);
            view_->fitInView(view_->scene()->sceneRect(), Qt::KeepAspectRatio);
            view_->scene()->update();
        }
    }
}
