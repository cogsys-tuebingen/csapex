/// HEADER
#include "simple_sink.h"

/// PROJECT
#include <evaluator/messages_default.hpp>
#include <designer/connector_in.h>
#include <designer/connector_out.h>
#include <designer/box.h>
#include <opencv2/opencv.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QLabel>

PLUGINLIB_EXPORT_CLASS(vision_plugins::SimpleSink, vision_evaluator::BoxedObject);


using namespace vision_evaluator;
using namespace vision_plugins;

SimpleSink::SimpleSink()
    : input_(NULL),output_(NULL), sunk(0)
{
}

void SimpleSink::fill(QBoxLayout *layout)
{
    if(input_ == NULL || output_ == NULL) {
        /// add input
        input_ = new ConnectorIn(box_, 0);
        box_->addInput(input_);

        assert(QObject::connect(input_, SIGNAL(messageArrived(ConnectorIn*)), this, SLOT(messageArrived(ConnectorIn*))));

        /// add output
        output_ = new ConnectorOut(box_, 0);
        box_->addOutput(output_);

        /// debug output
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

    CvMatMessage::Ptr m = boost::shared_dynamic_cast<CvMatMessage>(source->getMessage());

    std::vector<cv::Mat> channels;
    cv::split(m->value, channels);
    for(uint i = 0 ; i < channels.size() ; i++) {
        cv::equalizeHist(channels[i], channels[i]);
    }
    cv::merge(channels, m->value);

    /// create msg
    CvMatMessage::Ptr m1(new CvMatMessage);

    output_->publish(m);
}
