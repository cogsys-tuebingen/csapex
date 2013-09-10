/// HEADER
#include "filter_histogram.h"

/// PROJECT
#include <utils/LibCvTools/histogram.hpp>

#include <csapex_vision/cv_mat_message.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/box.h>
#include <opencv2/opencv.hpp>
#include <csapex/utility/qt_helper.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QWidget>
#include <QComboBox>
#include <QCheckBox>
#include <QSpinBox>

PLUGINLIB_EXPORT_CLASS(csapex::Histogram, csapex::BoxedObject)

using namespace csapex;
using namespace connection_types;

Histogram::Histogram() :
    input_(NULL), output_histogram_(NULL),container_bin_counts_(NULL),channel_count_(0)
{
    colors_.push_back(cv_histogram::COLOR_WHITE);
    colors_.push_back(cv_histogram::COLOR_GREEN);
    colors_.push_back(cv_histogram::COLOR_CYAN);
    colors_.push_back(cv_histogram::COLOR_RED);

    /// FIXME: this should be done in a CorePlugin!!!!
    Tag::createIfNotExists("Analysis");
    addTag(Tag::get("Analysis"));
}

Memento::Ptr Histogram::getState() const
{
    return boost::shared_ptr<State>(new State(state_));
}

void Histogram::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state_ = *m;
    channel_count_ = state_.channel_count;
    Q_EMIT modelChanged();
    for(int i = 0 ; i < channel_count_ ; i++) {
        bin_counts_[i]->setValue(m->bin_counts[i]);
    }

    slide_zoom_->setDoubleValue(state_.zoom);
}


void Histogram::fill(QBoxLayout *layout)
{
    if(input_ == NULL || output_histogram_ == NULL) {
        /// add input
        input_ = new ConnectorIn(box_, 0);
        box_->addInput(input_);

        /// add output
        output_histogram_ = new ConnectorOut(box_,0);
        box_->addOutput(output_histogram_);

        /// add sliders and stuff
        QHBoxLayout *zoom_layout = new QHBoxLayout;
        slide_zoom_ = QtHelper::makeDoubleSlider(zoom_layout, "Zoom ", 1.0, -20.0, 20.0, 0.01);
        container_zoom_ = QtHelper::wrapLayout(zoom_layout);
        layout->addWidget(container_zoom_);

    }
}

void Histogram::messageArrived(ConnectorIn *source)
{
    CvMatMessage::Ptr m = boost::shared_dynamic_cast<CvMatMessage>(source->getMessage());
    CvMatMessage::Ptr histogram(new CvMatMessage);

    if(m->value.channels() != channel_count_) {
        channel_count_ = m->value.channels();
        Q_EMIT modelChanged();

        if(channel_count_ > colors_.size())
            colors_.push_back(randomColor());

    }

    updateState();

    if(bin_counts_.size() != state_.channel_count)
        return;

    cv::Mat bins, ranges;
    prepare(bins, ranges);

    std::vector<cv::MatND>  histograms;
    cv_histogram::full_channel_histogram(m->value, histograms, cv::Mat(), bins, ranges);

    cv::Mat histogram_img(600,800,CV_8UC3, cv::Scalar(0,0,0));
    cv_histogram::render_histogram<float>(histograms, bins, colors_, histogram_img, slide_zoom_->doubleValue());
    histogram->value = histogram_img;
    output_histogram_->publish(histogram);
}

void Histogram::updateDynamicGui(QBoxLayout *layout)
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
        bin_counts_.push_back(QtHelper::makeSlider(internal_layout, ch.str(), HISTOGRAM_BINS_STD, 1, HISTOGRAM_BINS_MAX));
    }
    container_bin_counts_ = QtHelper::wrapLayout(internal_layout);
    layout->addWidget(container_bin_counts_);
}

void Histogram::prepare(cv::Mat &bins, cv::Mat &ranges)
{
    cv_histogram::prepare_params(bins, ranges, channel_count_);
    for(int i = 0 ; i < channel_count_ ; i++) {
        bins.at<int>(i) = bin_counts_[i]->value();
    }
}

void Histogram::updateState()
{
    state_.bin_counts.clear();
    for(std::vector<QSlider*>::iterator it = bin_counts_.begin() ; it != bin_counts_.end() ; it++) {
        state_.bin_counts.push_back((*it)->value());
    }
    state_.channel_count = channel_count_;
    state_.zoom = slide_zoom_->doubleValue();
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

/// MEMENTO ------------------------------------------------------------------------------------
void Histogram::State::readYaml(const YAML::Node &node)
{
    const YAML::Node &values = node["bin_counts"];
    for(YAML::Iterator it = values.begin() ; it != values.end() ; it++) {
        int value;
        *it >> value;
        bin_counts.push_back(value);
    }
    node["channel_count"] >> channel_count;
    node["zoom"] >> zoom;
}

void Histogram::State::writeYaml(YAML::Emitter &out) const
{
    out << YAML::Key << "bin_counts" << YAML::Value << YAML::BeginSeq;
    for(std::vector<int>::const_iterator it = bin_counts.begin() ; it != bin_counts.end() ; it++) {
        out << *it;
    }
    out << YAML::EndSeq;
    out << YAML::Key << "channel_count" << YAML::Value << channel_count;
    out << YAML::Key << "zoom" << YAML::Value << zoom;
}
