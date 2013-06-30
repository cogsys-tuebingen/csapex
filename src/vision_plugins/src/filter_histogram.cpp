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
#include <QWidget>
#include <QComboBox>
#include <QCheckBox>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vision_evaluator::Histogram, vision_evaluator::BoxedObject)

using namespace vision_evaluator;

Histogram::Histogram() :
    input_(NULL),output_(NULL), output_histogram_(NULL)
{
    colors_.push_back(cv_histogram::COLOR_WHITE);
    colors_.push_back(cv_histogram::COLOR_GREEN);
    colors_.push_back(cv_histogram::COLOR_CYAN);
    colors_.push_back(cv_histogram::COLOR_RED);
}

Memento::Ptr Histogram::getState()
{
    boost::shared_ptr<State> memento(new State);
    *memento = state_;

    return memento;
}

void Histogram::setState(Memento::Ptr memento)
{
//    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
//    assert(m.get());

//    state_ = *m;



    //    Q_EMIT filter_changed();
}


void Histogram::fill(QBoxLayout *layout)
{
    layout_ = layout;

    if(input_ == NULL || output_ == NULL || output_histogram_ == NULL) {
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

        check_equal_  = new QCheckBox("Equalize");
        layout->addWidget(check_equal_);

        QHBoxLayout *zoom_layout = new QHBoxLayout;
        slide_ch_zoom_ = QtHelper::makeDoubleSlider(zoom_layout, "Zoom ", 1.0, 1.0, 20.0, 0.01);
        zoom_container_ = QtHelper::wrapLayout(zoom_layout);
        layout_->addWidget(zoom_container_);


    }
}

void Histogram::messageArrived(ConnectorIn *source)
{
    CvMatMessage::Ptr m = boost::shared_dynamic_cast<CvMatMessage>(source->getMessage());
    CvMatMessage::Ptr histogram(new CvMatMessage);

    if(check_equal_->isChecked())
        cv_histogram::full_channel_equalize(m->value, m->value);

    cv::Mat bins =   (cv::Mat_<int>(3,1)   <<  256 , 256, 256);
    cv::Mat ranges = (cv::Mat_<float>(6,1) <<  0.f, 256.f, 0.f, 256.f, 0.f, 256.f);

    std::vector<cv::MatND>  histograms;
    cv_histogram::single_channel_histogram(m->value, histograms, cv::Mat(), bins, ranges);

    cv::Mat histogram_img(600,800,CV_8UC3, cv::Scalar(0,0,0));
    cv_histogram::render_histogram(histograms, bins, colors_, histogram_img, slide_ch_zoom_->doubleValue());
    histogram->value = histogram_img;

    output_->publish(m);
    output_histogram_->publish(histogram);
}
