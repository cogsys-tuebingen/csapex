#include "filter_coloradjustment.h"

using namespace vision_evaluator;
/// SYSTEM
#include <QComboBox>
#include <QCheckBox>

/// COMPONENT
#include <vision_evaluator/qt_helper.hpp>
#include <evaluator/messages_default.hpp>
#include <designer/connector_in.h>
#include <designer/connector_out.h>
#include <designer/box.h>
#include <utils/LibCvTools/histogram.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vision_evaluator::ColorAdjustment, vision_evaluator::BoxedObject)

ColorAdjustment::ColorAdjustment() :
    input_(NULL),
    output_(NULL),
    active_preset_(NONE),
    channel_count_(0),
    container_ch_sliders_(NULL)
{
}

ColorAdjustment::~ColorAdjustment()
{
}

void ColorAdjustment::fill(QBoxLayout *parent)
{
    layout_ = parent;

    if(input_ == NULL || output_ == NULL) {
        /// add input
        input_ = new ConnectorIn(box_, 0);
        box_->addInput(input_);

        assert(QObject::connect(input_, SIGNAL(messageArrived(ConnectorIn*)), this, SLOT(messageArrived(ConnectorIn*))));

        /// add output
        output_ = new ConnectorOut(box_, 0);
        box_->addOutput(output_);

        layout_ = parent;

        check_normalize_ = new QCheckBox("Normalization Mode");
        parent->addWidget(check_normalize_);


        parent->addWidget(new QLabel("Channel Adjustment"));

        QComboBox *combo_preset = new QComboBox();
        combo_preset->addItem("STD");
        combo_preset->addItem("HSV");
        combo_preset->addItem("HSL");
        presets_.insert ( std::pair<QString,Preset>(QString("STD"), STD) );
        presets_.insert ( std::pair<QString,Preset>(QString("HSV"), HSV) );
        presets_.insert ( std::pair<QString,Preset>(QString("HSL"), HSL) );
        layout_->addWidget(combo_preset);

        setPreset(combo_preset->currentText());


        slide_lightness_ = QtHelper::makeSlider(layout_, "Lightness -/+", 0, -255, 255);

        QObject::connect(combo_preset, SIGNAL(currentIndexChanged(QString)), this, SLOT(setPreset(QString)));
    }
}


void ColorAdjustment::messageArrived(ConnectorIn *source)
{
    CvMatMessage::Ptr m = boost::shared_dynamic_cast<CvMatMessage>(source->getMessage());

    std::vector<cv::Mat> channels;
    cv::split(m->value, channels);

    if(channel_count_ != m->value.channels()) {
        channel_count_ = m->value.channels();
        updateSliders();
    }

    for(int i = 0 ; i < slider_pairs_.size() && i < channels.size() ; i++) {
        double min = slider_pairs_[i].first->doubleValue();
        double max = slider_pairs_[i].second->doubleValue();
        if(check_normalize_->isChecked()) {
            cv::normalize(channels[i], channels[i], max, min, cv::NORM_MINMAX);
        } else {
            cv::threshold(channels[i], channels[i], min, max, cv::THRESH_TOZERO);
            cv::threshold(channels[i], channels[i], max, max, cv::THRESH_TOZERO_INV);
        }

    }

    cv::merge(channels, m->value);
    addLightness(m->value);

    output_->publish(m);
}

void ColorAdjustment::setPreset(QString text)
{
    active_preset_  = presets_[text];
    updateSliders();
}

void ColorAdjustment::addLightness(cv::Mat &img)
{
    double      slide_value = slide_lightness_->value();
    slide_value = std::abs(slide_value);

    cv::Scalar  cv_value;
    cv_value[0] = slide_value;
    cv_value[1] = active_preset_ == HSL || active_preset_ == HSV ? 0 : slide_value;
    cv_value[2] = active_preset_ == HSL || active_preset_ == HSV ? 0 : slide_value;
    cv_value[3] = slide_value;

    cv::Mat     lightness(img.rows, img.cols, img.type(), cv_value);
    if(slide_lightness_->value() < 0) {
        cv::subtract(img, lightness, img);
    } else if(slide_lightness_->value() > 0) {
        cv::add(img, lightness, img);
    }
}

void ColorAdjustment::prepareSliderPair(QDoubleSlider *min, QDoubleSlider *max)
{
    std::pair<QDoubleSlider*, QDoubleSlider*> minMax;
    minMax.first  = min;
    minMax.second = max;
    QDoubleSlider::connect(minMax.first, SIGNAL(valueChanged(double)), minMax.second, SLOT(limitMin(double)));
    QDoubleSlider::connect(minMax.second, SIGNAL(valueChanged(double)), minMax.first, SLOT(limitMax(double)));
    slider_pairs_.push_back(minMax);
}

void ColorAdjustment::updateSliders()
{
    slider_pairs_.clear();
    QVBoxLayout *internal_layout;

    if(container_ch_sliders_ != NULL) {
        container_ch_sliders_->deleteLater();
    }

    internal_layout = new QVBoxLayout;

    for(int i = 0 ; i < channel_count_ ; i++) {
        std::stringstream ch;
        ch << i + 1;

        double ch_limit = 255.0;
        if(i == 0 && (active_preset_ == HSL || active_preset_ == HSV)) {
            ch_limit = 180.0;
        }

        QDoubleSlider *tmp_min = QtHelper::makeDoubleSlider(internal_layout, "Ch." + ch.str() + "min.", 0.0, 0.0, ch_limit, 0.01);
        QDoubleSlider *tmp_max = QtHelper::makeDoubleSlider(internal_layout, "Ch." + ch.str() + "max.", 255.0, 0.0, ch_limit, 0.01);
        prepareSliderPair(tmp_min, tmp_max);

    }

    container_ch_sliders_ = QtHelper::wrapLayout(internal_layout);
    layout_->addWidget(container_ch_sliders_);
}
