#include "filter_local_patterns.h"

/// COMPONENT
#include <csapex/qt_helper.hpp>
#include <utils/LibCvTools/histogram.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QComboBox>
#include <QTime>

PLUGINLIB_EXPORT_CLASS(vision_plugins::LocalPatterns, csapex::BoxedObject)

using namespace vision_plugins;
using namespace csapex;

LocalPatterns::LocalPatterns()
{
    colors_.push_back(cv_histogram::COLOR_WHITE);
    colors_.push_back(cv_histogram::COLOR_GREEN);
    colors_.push_back(cv_histogram::COLOR_CYAN);
    colors_.push_back(cv_histogram::COLOR_RED);

    setCategory("Analysis");
}

void LocalPatterns::filter(Mat &img, Mat &mask)
{
    if(img.type() != CV_8UC1&&
       img.type() != CV_8UC2&&
       img.type() != CV_8UC3&&
       img.type() != CV_8UC4) {
        throw std::runtime_error("Image has to consist of chars!");
    }

    std::vector<cv::Mat> channels;
    cv::split(img, channels);

    cv::Mat out;
    if(combo_pattern_->currentText() == "LBP") {
        std::vector<cv::Mat> channel_hists;
        std::vector<int>     bins;

        foreach(cv::Mat channel, channels) {
            QTime t = QTime::currentTime();
            lbp_.stdExtraction<uchar>(channel);
            Q_EMIT timeUpdate(QTime::currentTime().msec() - t.msec());
            channel_hists.push_back(lbp_.getHistogram());
            bins.push_back(256);
        }
        out = cv::Mat(600, 800, CV_8UC3, cv::Scalar::all(0));
        cv_histogram::render_histogram<int>(channel_hists, bins, colors_, out);
    }
    if(combo_pattern_->currentText() == "LTP") {
        std::vector<cv::Mat> channel_hists_pos;
        std::vector<cv::Mat> channel_hists_neg;
        std::vector<int>     bins;

        foreach(cv::Mat channel, channels) {
            QTime t = QTime::currentTime();
            ltp_.stdExtraction<uchar>(channel, state_.k);
            Q_EMIT timeUpdate(QTime::currentTime().msec() - t.msec());
            channel_hists_pos.push_back(ltp_.getPos());
            channel_hists_neg.push_back(ltp_.getNeg());
            bins.push_back(256);
        }

        out = cv::Mat(600, 1600, CV_8UC3, cv::Scalar::all(0));
        cv::Mat roi_pos(out, cv::Rect(0,0, 800, 600));
        cv::Mat roi_neg(out, cv::Rect(800,0, 800,600));
        cv_histogram::render_histogram<int>(channel_hists_pos, bins, colors_, roi_pos);
        cv_histogram::render_histogram<int>(channel_hists_neg, bins, colors_, roi_neg);
    }
    img = out;
}

void LocalPatterns::insert(QBoxLayout *parent)
{
    QLabel *combo_title = new QLabel("Pattern Type:");
    parent->addWidget(combo_title);

    combo_pattern_ = new QComboBox();
    combo_pattern_->addItem("LBP");
    combo_pattern_->addItem("LTP");
    parent->addWidget(combo_pattern_);

    slider_k_ = QtHelper::makeDoubleSlider(parent, "k", 5, 0, 1000, 1.0);
    slider_k_->setEnabled(false);

    time_ = new QLabel("Time: ");
    parent->addWidget(time_);

    connect(slider_k_,      SIGNAL(doubleValueChanged(double)), this, SLOT(update()));
    connect(combo_pattern_, SIGNAL(currentIndexChanged(int)), this, SLOT(update()));
    connect(this, SIGNAL(timeUpdate(double)), this, SLOT(updateTime(double)), Qt::QueuedConnection);
}

void LocalPatterns::update()
{
    slider_k_->setEnabled(combo_pattern_->currentText() == "LTP");
    state_.index = combo_pattern_->currentIndex();
    state_.k     = slider_k_->doubleValue();
}

void LocalPatterns::updateTime(double value)
{
    QString text = "Time: ";
    time_->setText(text + QString::number(value));
}

Memento::Ptr LocalPatterns::getState() const
{
    boost::shared_ptr<State> memento(new State);
    *memento = state_;

    return memento;
}

void LocalPatterns::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state_ = *m;
    combo_pattern_->setCurrentIndex(state_.index);
    slider_k_->setDoubleValue(state_.k);
    slider_k_->setEnabled(combo_pattern_->currentText() == "LTP");

}

void LocalPatterns::State::readYaml(const YAML::Node &node)
{
    node["index"] >> index;
    node["k"]     >> k;
}

void LocalPatterns::State::writeYaml(YAML::Emitter &out) const
{
    out << YAML::Key << "index" << YAML::Value << index;
    out << YAML::Key << "k"     << YAML::Value << k;
}
