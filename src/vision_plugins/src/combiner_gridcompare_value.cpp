#include "combiner_gridcompare_value.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::GridCompareValue, vision_evaluator::BoxedObject)

using namespace vision_evaluator;
using namespace cv_grid;

GridCompareValue::GridCompareValue() :
    GridCompare(StateValue::Ptr(new StateValue)),
    container_eps_slider_(0)
{
    private_state_ = dynamic_cast<StateValue*>(state_.get());
    assert(private_state_);

}

cv::Mat GridCompareValue::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    if(!img1.empty() && !img2.empty()) {
        if(img1.channels() != img2.channels())
            throw std::runtime_error("Channel count is not matching!");

        if(img1.channels() > 4)
            throw std::runtime_error("Channel limit 4!");

        if(img1.rows != img2.rows || img1.cols != img2.cols)
            throw std::runtime_error("Dimension is not matching!");

        if(private_state_->channel_count != img1.channels()) {
            private_state_->channel_count = img1.channels();
            Q_EMIT modelChanged();
        }

        if(private_state_->img_cols != img1.cols || private_state_->img_rows != img1.rows) {
            private_state_->img_cols = img1.cols;
            private_state_->img_rows = img1.rows;
            slide_height_->setMaximum(img1.rows);
            slide_width_->setMaximum(img1.cols);

        }

        //// TODO GRIDMAXIMUM

        if(eps_sliders_.size() == private_state_->channel_count) {
            GridScalar g1, g2;
            AttrScalar::Params p;

            p.eps    = private_state_->eps;
            p.ignore = private_state_->ignore;

            cv_grid::prepare_grid<AttrScalar>(g1, img1, private_state_->grid_height, private_state_->grid_width, p, mask1, 1.0);
            cv_grid::prepare_grid<AttrScalar>(g2, img2, private_state_->grid_height, private_state_->grid_width, p, mask2, 1.0);

            cv::Mat out(img1.rows + 40, img1.cols, CV_8UC3, cv::Scalar(0,0,0));
            render_grid(g1, g2, out);
            return out;
        }
    }

    return cv::Mat();

}

void GridCompareValue::updateDynamicGui(QBoxLayout *layout)
{
    QVBoxLayout *internal_layout;
    if(container_eps_slider_ != NULL) {
        container_eps_slider_->deleteLater();
    }
    internal_layout = new QVBoxLayout;
    for(int i = 0 ; i < private_state_->channel_count ; i++) {
        std::stringstream ch;
        ch << "Ch." << i << " eps";

        QDoubleSlider *slider = QtHelper::makeDoubleSlider(internal_layout, ch.str(), private_state_->eps[i], 0.0, 255.0, 0.01);
        QDoubleSlider::connect(slider, SIGNAL(valueChanged(int)), this, SLOT(updateState(int)));
        internal_layout->addWidget(slider);
        eps_sliders_.push_back(slider);
    }

    container_eps_slider_ = QtHelper::wrapLayout(internal_layout);
    layout->addWidget(container_eps_slider_);

}

Memento::Ptr GridCompareValue::getState() const
{
    StateValue::Ptr memento(new StateValue);
    *memento = *boost::dynamic_pointer_cast<StateValue>(state_);
    return memento;
}

void GridCompareValue::setState(Memento::Ptr memento)
{
    state_.reset(new StateValue);
    StateValue::Ptr s = boost::dynamic_pointer_cast<StateValue>(memento);
    assert(s.get());
    *boost::dynamic_pointer_cast<StateValue>(state_) = *s;
    assert(state_.get());
    private_state_ = boost::dynamic_pointer_cast<StateValue>(state_).get();
    assert(private_state_);

    slide_height_->setValue(private_state_->grid_height);
    slide_width_->setValue(private_state_->grid_width);

    Q_EMIT modelChanged();
}

void GridCompareValue::updateState(int i)
{
    private_state_->grid_width  = slide_width_->value();
    private_state_->grid_height = slide_height_->value();
    prepareParams(private_state_->eps, private_state_->ignore);
}

void GridCompareValue::fill(QBoxLayout *layout)
{
    GridCompare::fill(layout);

    connect(slide_height_, SIGNAL(valueChanged(int)), this, SLOT(updateState(int)));
    connect(slide_width_, SIGNAL(valueChanged(int)), this, SLOT(updateState(int)));
}

void GridCompareValue::prepareParams(cv::Scalar &eps, cv::Vec<bool, 4> &ignore)
{
    for(int i = 0 ; i < eps_sliders_.size() ; i++) {
        eps[i] = eps_sliders_[i]->doubleValue();
        if(eps[i] == 255.0)
            ignore[i] = true;
    }
}

/// MEMENTO ------------------------------------------------------------------------------------
GridCompareValue::StateValue::StateValue() :
    GridCompare::State(),
    eps(cv::Scalar::all(0.0)),
    ignore(false, false, false, false)
{
}

void GridCompareValue::StateValue::readYaml(const YAML::Node &node)
{
    GridCompare::State::readYaml(node);

    const YAML::Node &_eps = node["eps"];
    int i = 0;
    for(YAML::Iterator it = _eps.begin() ; it != _eps.end() ; it++, i++) {
        *it >> eps[i];
    }
}

void GridCompareValue::StateValue::writeYaml(YAML::Emitter &out) const
{
    GridCompare::State::writeYaml(out);
    out << YAML::Key << "eps" << YAML::Value << YAML::BeginSeq;
    for(int i = 0 ; i < 4 ; i++) {
        out << eps[i];
    }
    out << YAML::EndSeq;

}