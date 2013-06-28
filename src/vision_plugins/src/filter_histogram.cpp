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

        slide_lightness_ = QtHelper::makeSlider(layout_, "Lightness -/+", 0, -255, 255);

        check_zoom_ = new QCheckBox("Scale");
        layout->addWidget(check_zoom_);

        QHBoxLayout *zoom_layout = new QHBoxLayout;
        slide_ch_zoom_ = QtHelper::makeDoubleSlider(zoom_layout, "Zoom ", 1.0, 1.0, 20.0, 0.01);
        zoom_container_ = QtHelper::wrapLayout(zoom_layout);
        layout_->addWidget(zoom_container_);

        check_ch_norm_  = new QCheckBox("ChannelNormalization");
        layout->addWidget(check_ch_norm_);

        combo_ch_norm_ = new QComboBox();
        combo_ch_norm_->addItem("RGB");
        combo_ch_norm_->addItem("BGR");
        combo_ch_norm_->addItem("HSL");
        combo_ch_norm_->addItem("HSV");
        presets_.insert ( std::pair<QString,Preset>(QString("RGB"), RGB) );
        presets_.insert ( std::pair<QString,Preset>(QString("BGR"), BGR) );
        presets_.insert ( std::pair<QString,Preset>(QString("HSV"), HSV) );
        presets_.insert ( std::pair<QString,Preset>(QString("HSL"), HSL) );
        layout->addWidget(combo_ch_norm_);
        selectedPreset(combo_ch_norm_->currentText());

        zoom_container_->setHidden(true);
        slide_ch_container_->setHidden(true);
        combo_ch_norm_->setHidden(true);
        slide_ch_zoom_->setHidden(true);

        QComboBox::connect(combo_ch_norm_, SIGNAL(currentIndexChanged(QString)), this, SLOT(selectedPreset(QString)));
        QCheckBox::connect(check_ch_norm_, SIGNAL(clicked(bool)), this, SLOT(enableNorm(bool)));
        QCheckBox::connect(check_zoom_, SIGNAL(clicked(bool)), this, SLOT(enableScale(bool)));
    }
}

void Histogram::messageArrived(ConnectorIn *source)
{
    bool channel_norm = check_ch_norm_->isChecked();
    CvMatMessage::Ptr m = boost::shared_dynamic_cast<CvMatMessage>(source->getMessage());
    CvMatMessage::Ptr histogram(new CvMatMessage);

    if(check_equal_->isChecked())
        cv_histogram::full_channel_equalize(m->value, m->value);

    if(channel_norm) {
        std::vector<double> norm_factors;
        for(int i = 0 ; i < channels_.size() ; i++) {
            if(i % 2 == 0) {
                channels_[i]->setDoubleValue(std::min(channels_[i + 1]->doubleValue(), channels_[i]->doubleValue()));
            } else {
                channels_[i]->setDoubleValue(std::max(channels_[i - 1]->doubleValue(), channels_[i]->doubleValue()));
            }

            norm_factors.push_back(channels_[i]->doubleValue());

        }

        cv_histogram::normalize<cv::NORM_MINMAX>(m->value, m->value, norm_factors);
    }

    addLightness(m->value);

    int   ch1_bin   = active_preset_ == HSV || active_preset_ == HSL ? 180 : 256;
    float ch1_range = active_preset_ == HSV || active_preset_ == HSL ? 180.f : 256.f;

    cv::Mat bins =   (cv::Mat_<int>(3,1)   <<  ch1_bin , 256, 256);
    cv::Mat ranges = (cv::Mat_<float>(6,1) <<  0.f, ch1_range, 0.f, 256.f, 0.f, 256.f);

    std::vector<cv::MatND>  histograms;
    cv_histogram::single_channel_histogram(m->value, histograms, cv::Mat(), bins, ranges);

    cv::Mat histogram_img(600,800,CV_8UC3, cv::Scalar(0,0,0));
    cv_histogram::render_histogram(histograms, bins, colors_, histogram_img, slide_ch_zoom_->doubleValue());
    histogram->value = histogram_img;

    output_->publish(m);
    output_histogram_->publish(histogram);
}

void Histogram::selectedPreset(QString text)
{
    channels_.clear();
    colors_.clear();
    QVBoxLayout *layout;

    if(slide_ch_container_ != NULL) {
        slide_ch_container_->deleteLater();
    }

    layout = new QVBoxLayout;
    slide_ch_container_ = QtHelper::wrapLayout(layout);

    Preset preset = presets_[text];

    if(preset == RGB || preset == BGR) {
        std::string ch1, ch3;
        cv::Scalar  cch1, cch3;

        if(preset == RGB) {
            ch1 = "R";
            ch3 = "B";
            cch1 = cv_histogram::COLOR_RED;
            cch3 = cv_histogram::COLOR_BLUE;
        } else {
            ch1 = "B";
            ch3 = "R";
            cch1 = cv_histogram::COLOR_BLUE;
            cch3 = cv_histogram::COLOR_RED;
        }
        channels_.push_back(QtHelper::makeDoubleSlider(layout, "min. " + ch1, 0.0, 0.0, 255.0, 0.01));
        channels_.push_back(QtHelper::makeDoubleSlider(layout, "max. " + ch1, 255.0, 0.0, 255.0, 0.01));
        channels_.push_back(QtHelper::makeDoubleSlider(layout, "min. G", 0.0, 0.0, 255.0, 0.01));
        channels_.push_back(QtHelper::makeDoubleSlider(layout, "max. G", 255.0, 0.0, 255.0, 0.01));
        channels_.push_back(QtHelper::makeDoubleSlider(layout, "min. " + ch3, 0.0, 0.0, 255.0, 0.01));
        channels_.push_back(QtHelper::makeDoubleSlider(layout, "max. " + ch3, 255.0, 0.0, 255.0, 0.01));
        colors_.push_back(cch1);
        colors_.push_back(cv_histogram::COLOR_GREEN);
        colors_.push_back(cch3);
    }

    if(preset == HSV || preset == HSL) {
        std::string ch2;
        std::string ch3;

        if(preset == HSV) {
            ch2 = "S";
            ch3 = "V";
        } else {
            ch2 = "L";
            ch3 = "S";
        }

        channels_.push_back(QtHelper::makeDoubleSlider(layout, "min. H", 0.0, 0.0, 180.0, 0.01));
        channels_.push_back(QtHelper::makeDoubleSlider(layout, "max. H", 90.0, 0.0, 180.0, 0.01));
        channels_.push_back(QtHelper::makeDoubleSlider(layout, "min. " + ch2, 0.0, 0.0, 255.0, 0.01));
        channels_.push_back(QtHelper::makeDoubleSlider(layout, "max. " + ch2, 127.0, 0.0, 255.0, 0.01));
        channels_.push_back(QtHelper::makeDoubleSlider(layout, "min. " + ch3, 0.0, 0.0, 255.0, 0.01));
        channels_.push_back(QtHelper::makeDoubleSlider(layout, "max. " + ch3, 127.0, 0.0, 255.0, 0.01));
        colors_.push_back(cv_histogram::COLOR_WHITE);
        colors_.push_back(cv_histogram::COLOR_CYAN);
        colors_.push_back(cv_histogram::COLOR_RED);
    }

    active_preset_ = preset;
    layout_->addWidget(slide_ch_container_);
}

void Histogram::enableNorm(bool value)
{
    combo_ch_norm_->setHidden(!value);
    slide_ch_container_->setHidden(!value);
}

void Histogram::enableScale(bool value)
{
    zoom_container_->setHidden(!value);
}

void Histogram::addLightness(cv::Mat &img)
{
    double      slide_value = slide_lightness_->value();
    slide_value = std::abs(slide_value);


    cv::Scalar  light_value;
    light_value[0] = slide_value;
    light_value[1] = active_preset_ == HSL || active_preset_ == HSV ? 0 : slide_value;
    light_value[2] = active_preset_ == HSL || active_preset_ == HSV ? 0 : slide_value;

    cv::Mat     lightness(img.rows, img.cols, img.type(), light_value);
    if(slide_lightness_->value() < 0) {
        cv::subtract(img, lightness, img);
    } else if(slide_lightness_->value() > 0) {
        cv::add(img, lightness, img);
    }
}
