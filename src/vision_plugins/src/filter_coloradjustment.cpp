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
    slide_ch_container_(NULL)
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

        limit_ = new QCheckBox("Normalization Mode");
        parent->addWidget(limit_);


        parent->addWidget(new QLabel("Channel Adjustment"));

        QComboBox *combo_preset = new QComboBox();
        combo_preset->addItem("1 Channel");
        combo_preset->addItem("2 Channel");
        combo_preset->addItem("3 Channel");
        combo_preset->addItem("4 Channel");
        combo_preset->addItem("HSV");
        combo_preset->addItem("HSL");
        presets_.insert ( std::pair<QString,Preset>(QString("1 Channel"), CH1) );
        presets_.insert ( std::pair<QString,Preset>(QString("2 Channel"), CH2) );
        presets_.insert ( std::pair<QString,Preset>(QString("3 Channel"), CH3) );
        presets_.insert ( std::pair<QString,Preset>(QString("4 Channel"), CH4) );
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

    for(int i = 0 ; i < channel_limits_.size() && i < channels.size() ; i++) {
        double min = channel_limits_[i].first->doubleValue();
        double max = channel_limits_[i].second->doubleValue();
        if(limit_->isChecked()) {
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
    channel_limits_.clear();
    QVBoxLayout *layout;

    if(slide_ch_container_ != NULL) {
        slide_ch_container_->deleteLater();
    }

    layout = new QVBoxLayout;
    Preset preset = presets_[text];

    for(int i = 0 ; i < channel_count(preset) ; i++) {
        std::stringstream ch;
        ch << i;

        double ch_limit = 255.0;
        if(i == 0 && (preset == HSL || preset == HSV)) {
            ch_limit = 180.0;
        }

        QDoubleSlider *tmp_min = QtHelper::makeDoubleSlider(layout, "min. " + ch.str(), 0.0, 0.0, ch_limit, 0.01);
        QDoubleSlider *tmp_max = QtHelper::makeDoubleSlider(layout, "max. " + ch.str(), 255.0, 0.0, ch_limit, 0.01);
        insertPair(tmp_min, tmp_max);

    }

    slide_ch_container_ = QtHelper::wrapLayout(layout);
    active_preset_ = preset;
    layout_->addWidget(slide_ch_container_);

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

void ColorAdjustment::connectPair(const std::pair<QDoubleSlider *, QDoubleSlider *> &pair)
{
    QDoubleSlider::connect(pair.first, SIGNAL(valueChanged(double)), pair.second, SLOT(limitMin(double)));
    QDoubleSlider::connect(pair.second, SIGNAL(valueChanged(double)), pair.first, SLOT(limitMax(double)));
}

void ColorAdjustment::insertPair(QDoubleSlider *min, QDoubleSlider *max)
{
    std::pair<QDoubleSlider*, QDoubleSlider*> minMax;
    minMax.first  = min;
    minMax.second = max;
    connectPair(minMax);
    channel_limits_.push_back(minMax);
}

int ColorAdjustment::channel_count(Preset p)
{
    switch(p) {
    case CH1:
        return 1;
    case CH2:
        return 2;
    case CH3:
        return 3;
    case HSL:
        return 3;
    case HSV:
        return 3;
    case CH4:
        return 4;
    }
}
