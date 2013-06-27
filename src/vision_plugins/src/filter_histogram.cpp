/// HEADER
#include "filter_histogram.h"

/// PROJECT
#include <utils/LibCvTools/histogram.hpp>

#include <evaluator/messages_default.hpp>
#include <designer/connector_in.h>
#include <designer/connector_out.h>
#include <designer/box.h>
#include <opencv2/opencv.hpp>
#include <vision_evaluator/qt_helper.hpp>
#include <QCheckBox>
#include <QGroupBox>
#include <QWidget>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vision_evaluator::Histogram, vision_evaluator::BoxedObject)

using namespace vision_evaluator;

Histogram::Histogram() :
    input_(NULL),output_(NULL)
{
}

void Histogram::fill(QBoxLayout *layout)
{
    if(input_ == NULL || output_ == NULL) {
        /// add input
        input_ = new ConnectorIn(box_, 0);
        box_->addInput(input_);

        assert(QObject::connect(input_, SIGNAL(messageArrived(ConnectorIn*)), this, SLOT(messageArrived(ConnectorIn*))));

        /// add output
        output_ = new ConnectorOut(box_, 0);
        box_->addOutput(output_);

        output_histogram_ = new ConnectorOut(box_,1);
        box_->addOutput(output_histogram_);

        /// add sliders and stuff
        QCheckBox   *checkbox_scale  = new QCheckBox("scale");
        layout->addWidget(checkbox_scale);
        QtHelper::makeDoubleSlider(layout, "scale ", 500.0, 0, 1000.0, 0.01);

        QCheckBox   *checkbox_norma  = new QCheckBox("normalize");
        layout->addWidget(checkbox_norma);
        QtHelper::makeDoubleSlider(layout, "ch 1", 127.0, 0.0, 255.0, 0.01);
        QtHelper::makeDoubleSlider(layout, "ch 2", 127.0, 0.0, 255.0, 0.01);
        QtHelper::makeDoubleSlider(layout, "ch 3", 127.0, 0.0, 255.0, 0.01);

        QCheckBox   *checkbox_equal  = new QCheckBox("equalize");
        layout->addWidget(checkbox_equal);




    }
}

void Histogram::messageArrived(ConnectorIn *source)
{

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
