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
    input_(NULL),output_(NULL), output_histogram_(NULL),slide_ch_container_(NULL)
{
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

        check_ch_norm_  = new QCheckBox("Channelnormalization");
        layout->addWidget(check_ch_norm_);

        combo_ch_norm_ = new QComboBox();
        combo_ch_norm_->addItem("RGB");
        combo_ch_norm_->addItem("BGR");
        combo_ch_norm_->addItem("HSL");
        combo_ch_norm_->addItem("HSV");
        presets_.insert ( std::pair<QString,Preset>(QString("RGB"), RGB) );
        presets_.insert ( std::pair<QString,Preset>(QString("BGR"), RGB) );
        presets_.insert ( std::pair<QString,Preset>(QString("HSL"), RGB) );
        presets_.insert ( std::pair<QString,Preset>(QString("HSV"), RGB) );
        layout->addWidget(combo_ch_norm_);


        QHBoxLayout* scale_layout = new QHBoxLayout;
        slide_ch_scale_ = QtHelper::makeDoubleSlider(scale_layout, "Scale ", 5.0, 0, 10.0, 0.01);
        slide_ch_scale_container_ = QtHelper::wrapLayout<QHBoxLayout>(scale_layout);
        layout->addWidget(slide_ch_scale_container_);

        slide_ch_scale_container_->setHidden(true);
        combo_ch_norm_->setHidden(true);

        QComboBox::connect(combo_ch_norm_, SIGNAL(currentIndexChanged(QString)), this, SLOT(selectedPreset(QString)));
        QCheckBox::connect(check_ch_norm_, SIGNAL(clicked(bool)), this, SLOT(enableNorm(bool)));
    }
}

void Histogram::messageArrived(ConnectorIn *source)
{
    bool channel_norm = check_ch_norm_->isChecked();
    CvMatMessage::Ptr m = boost::shared_dynamic_cast<CvMatMessage>(source->getMessage());
    CvMatMessage::Ptr histogram(new CvMatMessage);

    if(check_equal_->isChecked())
        cv::equalizeHist(m->value, m->value);

    if(channel_norm) {
        std::vector<double> norm_factors;
        norm_factors.push_back(slide_ch_scale_->doubleValue() * HSV || HSL ? 179.0 : 255.0);
        norm_factors.push_back(slide_ch_scale_->doubleValue() * 255.0);
        norm_factors.push_back(slide_ch_scale_->doubleValue() * 255.0);
        cv_histogram::normalize<cv::NORM_MINMAX>(m->value, m->value, norm_factors);
    }

    if(channel_norm) {
        std::vector<double> norm_factors;
        norm_factors.push_back(channels_[0]->doubleValue());
        norm_factors.push_back(channels_[1]->doubleValue());
        norm_factors.push_back(channels_[2]->doubleValue());
        cv_histogram::normalize<cv::NORM_L2>(m->value, m->value, norm_factors);
    }

    std::vector<cv::Mat> histograms;
    std::vector<int>     bins;
    std::vector<int>     ranges;
    bins.push_back( HSV || HSL ? 180 : 256 );
    bins.push_back(256);
    bins.push_back(256);
    ranges.push_back(HSV || HSL ? 179 : 255);
    ranges.push_back(255);
    ranges.push_back(255);
    cv_histogram::generate_histogram(m->value, histograms, bins, ranges, std::vector<float>());

    cv::Mat histogram_img(480,640,CV_8UC3, cv::Scalar(0,0,0));
    cv_histogram::render_histogram(histograms, bins, colors, histogram_img);
    histogram->value = histogram_img;

    output_->publish(m);
    output_histogram_->publish(histogram);
}

void Histogram::selectedPreset(QString text)
{
    if(slide_ch_container_ != NULL) {
        layout_->removeWidget(slide_ch_container_);
        delete slide_ch_container_;
    }

    QBoxLayout *container_layout = new QVBoxLayout;

    switch(presets_[text]) {
    case RGB:
        QtHelper::makeDoubleSlider(container_layout, "R", 127.0, 0.0, 255.0, 0.01);
        QtHelper::makeDoubleSlider(container_layout, "G", 127.0, 0.0, 255.0, 0.01);
        QtHelper::makeDoubleSlider(container_layout, "B", 127.0, 0.0, 255.0, 0.01);
        colors.push_back(cv_histogram::COLOR_RED);
        colors.push_back(cv_histogram::COLOR_GREEN);
        colors.push_back(cv_histogram::COLOR_BLUE);
        active_preset_ = RGB;
        break;
    case BGR:
        QtHelper::makeDoubleSlider(container_layout, "B", 127.0, 0.0, 255.0, 0.01);
        QtHelper::makeDoubleSlider(container_layout, "G", 127.0, 0.0, 255.0, 0.01);
        QtHelper::makeDoubleSlider(container_layout, "R", 127.0, 0.0, 255.0, 0.01);
        colors.push_back(cv_histogram::COLOR_BLUE);
        colors.push_back(cv_histogram::COLOR_GREEN);
        colors.push_back(cv_histogram::COLOR_RED);
        active_preset_ = BGR;
        break;
    case HSV:
        QtHelper::makeDoubleSlider(container_layout, "H", 90.0, 0.0, 180.0, 0.01);
        QtHelper::makeDoubleSlider(container_layout, "S", 127.0, 0.0, 255.0, 0.01);
        QtHelper::makeDoubleSlider(container_layout, "V", 127.0, 0.0, 255.0, 0.01);
        colors.push_back(cv_histogram::COLOR_WHITE);
        colors.push_back(cv_histogram::COLOR_CYAN);
        colors.push_back(cv_histogram::COLOR_RED);
        active_preset_ = HSV;
        break;
    case HSL:
        QtHelper::makeDoubleSlider(container_layout, "H", 90.0, 0.0, 180.0, 0.01);
        QtHelper::makeDoubleSlider(container_layout, "L", 127.0, 0.0, 255.0, 0.01);
        QtHelper::makeDoubleSlider(container_layout, "S", 127.0, 0.0, 255.0, 0.01);
        colors.push_back(cv_histogram::COLOR_WHITE);
        colors.push_back(cv_histogram::COLOR_CYAN);
        colors.push_back(cv_histogram::COLOR_RED);
        active_preset_ = HSL;
        break;
    case NONE:
        break;
    default:
        std::cerr << "Problems adding sliders!" << std::endl;
    }

    slide_ch_container_ = new QWidget;
    slide_ch_container_->setLayout(container_layout);
    layout_->addWidget(slide_ch_container_);
    layout_->update();
}

void Histogram::enableNorm(bool value)
{
    combo_ch_norm_->setHidden(!value);
    slide_ch_scale_container_->setHidden(!value);

    if(slide_ch_container_ != NULL)
        slide_ch_container_->setHidden(!value);

    layout_->update();
}
