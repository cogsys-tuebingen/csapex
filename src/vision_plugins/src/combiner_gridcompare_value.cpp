#include "combiner_gridcompare_value.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::GridCompareValue, vision_evaluator::ImageCombiner)

using namespace vision_evaluator;
using namespace cv_grid;

GridCompareValue::GridCompareValue() :
    container_eps_slider_(0)
{
    state_.channel_count = 0;
    state_.restored = false;
}

cv::Mat GridCompareValue::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    if(!img1.empty() && !img2.empty()) {
        if(img1.channels() != img2.channels())
            throw std::runtime_error("Channel count is not matching!");
        if(img1.channels() > 4)
            throw std::runtime_error("Channel limit 4!");

        if(state_.channel_count != img1.channels()) {
            state_.channel_count = img1.channels();
            Q_EMIT modelChanged();
        }

        if(eps_sliders_.size() == state_.channel_count) {
            GridScalar g1, g2;
            AttrScalar::Params p;
            prepareParams(p.eps, p.ignore);


            /// MEMENTO
            state_.eps = p.eps;
            int height = slide_height_->value();
            int width =  slide_width_->value();

            state_.grid_width = width;
            state_.grid_height = height;

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
    for(int i = 0 ; i < state_.channel_count ; i++) {
        std::stringstream ch;
        ch << "Ch." << i << " eps";

        double default_eps = 0.0;

        if(state_.restored) {
            default_eps = state_.eps[i];
        }

        QDoubleSlider *slider = QtHelper::makeDoubleSlider(internal_layout, ch.str(), default_eps, 0.0, 255.0, 0.01);
        internal_layout->addWidget(slider);
        eps_sliders_.push_back(slider);
    }

    if(state_.restored) {
        slide_height_->setValue(state_.grid_height);
        slide_width_->setValue(state_.grid_height);
        state_.restored = false;
    } else {
        state_.eps = cv::Scalar::all(0.0);
    }

    container_eps_slider_ = QtHelper::wrapLayout(internal_layout);
    layout->addWidget(container_eps_slider_);

}

Memento::Ptr GridCompareValue::getState() const
{
    boost::shared_ptr<State> memento(new State);
    *memento = state_;

    return memento;
}

void GridCompareValue::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());
    state_ = *m;
    Q_EMIT modelChanged();
}



void GridCompareValue::fill(QBoxLayout *layout)
{
    GridCompare::fill(layout);
}

void GridCompareValue::prepareParams(cv::Scalar &eps, cv::Vec<bool, 4> &ignore)
{
    for(int i = 0 ; i < eps_sliders_.size() ; i++) {
        eps[i] = eps_sliders_[i]->doubleValue();
        if(eps[i] == 255.0)
            ignore[i] = true;
    }
}
