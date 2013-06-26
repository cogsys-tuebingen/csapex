/// HEADER
#include "simple_sink.h"

/// PROJECT
#include <designer/connector_in.h>
#include <designer/box.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QLabel>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::SimpleSink, vision_evaluator::BoxedObject);


using namespace vision_evaluator;

SimpleSink::SimpleSink()
    : input_(NULL), sunk(0)
{
}

void SimpleSink::fill(QBoxLayout *layout)
{
    if(input_ == NULL) {
        input_ = new ConnectorIn(box_, 0);
        box_->addInput(input_);

        assert(QObject::connect(input_, SIGNAL(messageArrived(ConnectorIn*)), this, SLOT(messageArrived(ConnectorIn*))));

        label = new QLabel;
        layout->addWidget(label);
    }
}

void SimpleSink::messageArrived(ConnectorIn *source)
{
    ++sunk;

    std::stringstream txt;
    txt << "sunk: " << sunk;
    label->setText(txt.str().c_str());
}
