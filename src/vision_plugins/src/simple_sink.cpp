/// HEADER
#include "simple_sink.h"

/// PROJECT
#include <csapex_vision/cv_mat_message.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/box.h>
#include <opencv2/opencv.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QLabel>

PLUGINLIB_EXPORT_CLASS(csapex::SimpleSink, csapex::BoxedObject);


using namespace csapex;
using namespace connection_types;

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
