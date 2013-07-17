#include "combiner_gridcompare_value.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::GridCompareValue, vision_evaluator::ImageCombiner)

using namespace vision_evaluator;
using namespace cv_grid;

GridCompareValue::GridCompareValue() :
    GridCompare(State::Ptr(new State)),
    container_eps_slider_(0)
{
    private_state_ = dynamic_cast<State*>(state_.get());
    assert(private_state_);

}

cv::Mat GridCompareValue::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    if(!img1.empty() && !img2.empty()) {
        if(img1.channels() != img2.channels())
            throw std::runtime_error("Channel count is not matching!");
        if(img1.channels() > 4)
            throw std::runtime_error("Channel limit 4!");

        if(private_state_->channel_count != img1.channels()) {
            private_state_->channel_count = img1.channels();
            Q_EMIT modelChanged();
        }

        if(eps_sliders_.size() == private_state_->channel_count) {
            GridScalar g1, g2;
            AttrScalar::Params p;
            prepareParams(p.eps, p.ignore);


            /// MEMENTO
            private_state_->eps = p.eps;
            int height = slide_height_->value();
            int width =  slide_width_->value();

            cv_grid::prepare_grid<AttrScalar>(g1, img1, height, width, p, mask1, 1.0);
            cv_grid::prepare_grid<AttrScalar>(g2, img2, height, width, p, mask2, 1.0);

            cv::Mat out(img1.rows + 40, img1.cols, CV_8UC3, cv::Scalar(0,0,0));
            render_grid(g1, g2, out);
            return out;
        }
    }

    return cv::Mat();

}

void GridCompareValue::updateGui(QBoxLayout *layout)
{
    QVBoxLayout *internal_layout;
    if(container_eps_slider_ != NULL) {
        container_eps_slider_->deleteLater();
    }
    internal_layout = new QVBoxLayout;
    for(int i = 0 ; i < private_state_->channel_count ; i++) {
        std::stringstream ch;
        ch << "Ch." << i << " eps";

        double default_eps = 0.0;

        if(private_state_->restored) {
            default_eps = private_state_->eps[i];
        }

        QDoubleSlider *slider = QtHelper::makeDoubleSlider(internal_layout, ch.str(), default_eps, 0.0, 255.0, 0.01);
        internal_layout->addWidget(slider);
        eps_sliders_.push_back(slider);
    }

    if(private_state_->restored) {
        slide_height_->setValue(private_state_->grid_height);
        slide_width_->setValue(private_state_->grid_height);
        private_state_->restored = false;
    } else {
        private_state_->eps = cv::Scalar::all(0.0);
    }

    container_eps_slider_ = QtHelper::wrapLayout(internal_layout);
    layout->addWidget(container_eps_slider_);

}

Memento::Ptr GridCompareValue::getState() const
{
    State::Ptr memento(new State);
    *memento = *state_;

    return memento;
}

void GridCompareValue::setState(Memento::Ptr memento)
{
    state_.reset(new State);
    State::Ptr s = boost::dynamic_pointer_cast<State>(memento);
    assert(s.get());
    *state_ = *s;
    assert(state_.get());
    private_state_ = dynamic_cast<State*>(state_.get());
    assert(private_state_);
    Q_EMIT modelChanged();
}

void GridCompareValue::updateState()
{
    private_state_->grid_width = slide_width_->value();
    private_state_->grid_height = slide_height_->value();
}

void GridCompareValue::fill(QBoxLayout *layout)
{
    GridCompare::fill(layout);
    connect(slide_height_, SIGNAL(sliderReleased()), this, SLOT(updateState()));
    connect(slide_width_, SIGNAL(sliderReleased()), this, SLOT(updateState()));

}

void GridCompareValue::prepareParams(cv::Scalar &eps, cv::Vec<bool, 4> &ignore)
{
    for(int i = 0 ; i < eps_sliders_.size() ; i++) {
        eps[i] = eps_sliders_[i]->doubleValue();
        if(eps[i] == 255.0)
            ignore[i] = true;
    }
}

/// MEMENTO
void GridCompareValue::State::readYaml(const YAML::Node &node)
{
    GridCompare::State::readYaml(node);
    const YAML::Node &_eps = node["eps"];
    int i = 0;
    for(YAML::Iterator it = _eps.begin() ; it != _eps.end() ; it++, i++) {
        *it >> eps[i];
    }
}

void GridCompareValue::State::writeYaml(YAML::Emitter &out) const
{
    GridCompare::State::writeYaml(out);
    out << YAML::Key << "eps" << YAML::Value << YAML::BeginSeq;
    for(int i = 0 ; i < 4 ; i++) {
        out << eps[i];
    }
    out << YAML::EndSeq;

}
