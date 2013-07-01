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

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QWidget>
#include <QComboBox>
#include <QCheckBox>
#include <QSpinBox>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::Histogram, vision_evaluator::BoxedObject)

using namespace vision_evaluator;

Histogram::Histogram() :
    input_(NULL), output_histogram_(NULL), channel_count_(0)
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

    if(input_ == NULL || output_histogram_ == NULL) {
        /// add input
        input_ = new ConnectorIn(box_, 0);
        box_->addInput(input_);

        assert(QObject::connect(input_, SIGNAL(messageArrived(ConnectorIn*)), this, SLOT(messageArrived(ConnectorIn*))));

        /// add output
        output_histogram_ = new ConnectorOut(box_,0);
        box_->addOutput(output_histogram_);

        /// add sliders and stuff
        QHBoxLayout *zoom_layout = new QHBoxLayout;
        slide_zoom_ = QtHelper::makeDoubleSlider(zoom_layout, "Zoom ", 1.0, 1.0, 20.0, 0.01);
        container_zoom_ = QtHelper::wrapLayout(zoom_layout);
        layout_->addWidget(container_zoom_);

    }
}

void Histogram::messageArrived(ConnectorIn *source)
{
    CvMatMessage::Ptr m = boost::shared_dynamic_cast<CvMatMessage>(source->getMessage());
    CvMatMessage::Ptr histogram(new CvMatMessage);

    if(m->value.channels() != channel_count_) {
        channel_count_ = m->value.channels();
        updateSliders();

        if(channel_count_ > colors_.size())
            colors_.push_back(randomColor());

    }



    cv::Mat bins, ranges;
    prepare(bins, ranges);

    std::vector<cv::MatND>  histograms;
    cv_histogram::single_channel_histogram(m->value, histograms, cv::Mat(), bins, ranges);

    cv::Mat histogram_img(600,800,CV_8UC3, cv::Scalar(0,0,0));
    cv_histogram::render_histogram(histograms, bins, colors_, histogram_img, slide_zoom_->doubleValue());
    histogram->value = histogram_img;
    output_histogram_->publish(histogram);
}

void Histogram::prepare(cv::Mat &bins, cv::Mat &ranges)
{
    cv::Mat_<int>  b(channel_count_, 1);
    cv::Mat_<float>r(channel_count_ * 2, 1);

    for(int i = 0 ; i < channel_count_ ; i++) {
        b.at<int>(i) = bin_counts_[i]->value();
        r.at<float>(2 * i) = 0.f;
        r.at<float>(2 * i + 1) = 256.f;
    }

    bins = b;
    ranges = r;
}

void Histogram::updateSliders()
{
    bin_counts_.clear();
    QVBoxLayout *internal_layout;

    if(container_bin_counts_ != NULL) {
        container_bin_counts_->deleteLater();
    }

    internal_layout = new QVBoxLayout;
    for(int i = 0 ; i < channel_count_; i++) {
        std::stringstream ch;
        ch << "Ch." << i+1 << " bins";
        bin_counts_.push_back(QtHelper::makeSpinBox(internal_layout, ch.str(), HISTOGRAM_BINS_STD, 1, HISTOGRAM_BINS_MAX));
    }
    container_bin_counts_ = QtHelper::wrapLayout(internal_layout);
    layout_->addWidget(container_bin_counts_);
}

cv::Scalar Histogram::randomColor()
{
    srand(time(0));
    cv::Scalar s;
    s[0] = rand() % 256;
    s[1] = rand() % 256;
    s[2] = rand() % 256;
    return s;
}
