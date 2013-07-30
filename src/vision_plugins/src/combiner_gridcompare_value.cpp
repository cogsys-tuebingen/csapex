#include "combiner_gridcompare_value.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(csapex::GridCompareValue, csapex::BoxedObject)

using namespace csapex;
using namespace cv_grid;

GridCompareValue::GridCompareValue() :
    GridCompare(State::Ptr(new State)),
    container_eps_slider_(NULL)
{
    private_state_gcv_ = dynamic_cast<State*>(state_.get());
    assert(private_state_gcv_);
}


GridCompareValue::GridCompareValue(GridCompare::State::Ptr state) :
    GridCompare(state),
    container_eps_slider_(NULL)
{
    private_state_gcv_ = dynamic_cast<State*>(state_.get());
    assert(private_state_gcv_);
}

cv::Mat GridCompareValue::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    if(!img1.empty() && !img2.empty()) {
        /// PREPARE
        if(img1.channels() != img2.channels())
            throw std::runtime_error("Channel count is not matching!");
        if(img1.channels() > 4)
            throw std::runtime_error("Channel limit 4!");
        //        if(img1.rows != img2.rows || img1.cols != img2.cols)
        //            throw std::runtime_error("Dimension is not matching!");

        if(private_state_gcv_->channel_count != img1.channels()) {
            private_state_gcv_->channel_count = img1.channels();
            Q_EMIT modelChanged();
        }

        updateSliderMaxima(img1.cols, img1.rows);

        /// COMPUTE
        if(eps_sliders_.size() == private_state_gcv_->channel_count) {
            state_buffer_gcv_ = *private_state_gcv_;
            GridScalar g1, g2;
            prepareGrid(g1, img1, mask1, state_buffer_gcv_.grid_width, state_buffer_gcv_.grid_height);
            prepareGrid(g2, img2, mask2, state_buffer_gcv_.grid_width, state_buffer_gcv_.grid_height);

            cv::Mat out;
            render_grid(g1, g2, cv::Size(10,10), out);
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

    for(int i = 0 ; i < private_state_gcv_->channel_count ; i++) {
        std::stringstream ch;
        ch << "Ch." << i << " eps";

        QDoubleSlider *slider = QtHelper::makeDoubleSlider(internal_layout, ch.str(), private_state_gcv_->eps[i], 0.0, 255.0, 0.01);
        QDoubleSlider::connect(slider, SIGNAL(valueChanged(int)), this, SLOT(updateState(int)));
        internal_layout->addWidget(slider);
        eps_sliders_.push_back(slider);
    }

    container_eps_slider_ = QtHelper::wrapLayout(internal_layout);
    layout->addWidget(container_eps_slider_);

}

Memento::Ptr GridCompareValue::getState() const
{
    State::Ptr memento(new State);
    *memento = *boost::dynamic_pointer_cast<State>(state_);
    return memento;
}

void GridCompareValue::setState(Memento::Ptr memento)
{
    state_.reset(new State);
    State::Ptr s = boost::dynamic_pointer_cast<State>(memento);
    assert(s.get());
    *boost::dynamic_pointer_cast<State>(state_) = *s;
    assert(state_.get());
    private_state_gcv_ = boost::dynamic_pointer_cast<State>(state_).get();
    assert(private_state_gcv_);

    state_mutex_.lock();
    slide_height_->setValue(private_state_gcv_->grid_height);
    slide_width_->setValue(private_state_gcv_->grid_width);
    state_mutex_.unlock();

    Q_EMIT modelChanged();
}

void GridCompareValue::updateState(int i)
{
    if(state_mutex_.tryLock()) {
        private_state_gcv_->grid_width  = slide_width_->value();
        private_state_gcv_->grid_height = slide_height_->value();
        prepareParams(private_state_gcv_->eps, private_state_gcv_->ignore);
        state_mutex_.unlock();
    }
}

void GridCompareValue::fill(QBoxLayout *layout)
{
    GridCompare::fill(layout);
    connect(slide_height_, SIGNAL(valueChanged(int)), this, SLOT(updateState(int)));
    connect(slide_width_, SIGNAL(valueChanged(int)), this, SLOT(updateState(int)));
}

void GridCompareValue::prepareGrid(cv_grid::GridScalar &g, const cv::Mat &img, const cv::Mat &mask, const int width, const int height)
{
    AttrScalar::Params p;
    p.eps    = private_state_gcv_->eps;
    p.ignore = private_state_gcv_->ignore;
    cv_grid::prepare_grid<AttrScalar>(g, img, height, width, p, mask, 1.0);
}

void GridCompareValue::prepareParams(cv::Scalar &eps, cv::Vec<bool, 4> &ignore)
{
    for(int i = 0 ; i < eps_sliders_.size() ; i++) {
        eps[i] = eps_sliders_[i]->doubleValue();
        ignore[i] = eps[i] == 255.0;
    }
}

/// MEMENTO ------------------------------------------------------------------------------------
GridCompareValue::State::State() :
    GridCompare::State(),
    eps(cv::Scalar::all(0.0)),
    ignore(false, false, false, false)
{
}

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
