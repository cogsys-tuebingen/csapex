#include "combiner_gridheatmap_value.h"

/// COMPONENT
#include <csapex/connector_out.h>
#include <csapex_vision/messages_default.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QPushButton>

PLUGINLIB_EXPORT_CLASS(csapex::GridHeatMapValue, csapex::BoxedObject)

using namespace csapex;
using namespace QSignalBridges;
using namespace cv_grid;
using namespace connection_types;

GridHeatMapValue::GridHeatMapValue() :
    GridCompareValue(State::Ptr(new State)),
    run_state_(RESET)
{
    private_state_ghv_ = dynamic_cast<State*>(state_.get());
    assert(private_state_ghv_);
}

cv::Mat GridHeatMapValue::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    if(!img1.empty() && !img2.empty()) {
        /// PREPARE
        if(img1.channels() != img2.channels())
            throw std::runtime_error("Channel count is not matching!");
        if(img1.channels() > 4)
            throw std::runtime_error("Channel limit 4!");
        if(img1.rows > img2.rows || img1.cols > img2.cols)
            throw std::runtime_error("Image 1 must have smaller or euqal size!");


        if(private_state_gcv_->channel_count != img1.channels()) {
            private_state_gcv_->channel_count = img1.channels();
            Q_EMIT modelChanged();
        }

        updateSliderMaxima(img1.cols, img1.rows, img2.cols, img2.rows);

        /// COMPUTE
        if(eps_sliders_.size() == private_state_gcv_->channel_count && run_state_ == RESET) {
            state_buffer_ghv_ = *private_state_ghv_;

            run_state_ = RUNNING;

            GridScalar g1, g2;
            prepareGrid(g1, img1, mask1, state_buffer_ghv_.grid_width, state_buffer_ghv_.grid_height);
            prepareGrid(g2, img2, mask2, state_buffer_ghv_.grid_width_add1, state_buffer_ghv_.grid_height_add1);

            cv::Mat  values;
            cv::Size block_size(10,10);
            cv::Mat  out;
            HeatMapCompIterator<GridScalar>it(g1, g2);

            while(it.iterate(values)) {
                render_heatmap_row(values,block_size, it.lastFinishedRow(), out);

                if(run_state_ == RESET)
                    return cv::Mat();

                CvMatMessage::Ptr img_msg_result(new CvMatMessage);
                img_msg_result->value = out;
                output_img_->publish(img_msg_result);
            }

            buffered_image_ = out;
            run_state_ = BUFFERING;
        }

        if(run_state_ == BUFFERING)
            return buffered_image_;
    }

    return cv::Mat();

}

void GridHeatMapValue::updateState(int value)
{
    if(state_mutex_.tryLock()) {
        private_state_ghv_->grid_width       = slide_width_->value();
        private_state_ghv_->grid_height      = slide_height_->value();
        private_state_ghv_->grid_width_add1  = slide_width_add1_->value();
        private_state_ghv_->grid_height_add1 = slide_height_add1_->value();
        GridCompareValue::prepareParams(private_state_ghv_->eps,private_state_ghv_->ignore);
        state_mutex_.unlock();
    }
}

void GridHeatMapValue::reset()
{
    run_state_ = RESET;
}

void GridHeatMapValue::addSliders(QBoxLayout *layout)
{
    slide_width_       = QtHelper::makeSlider(layout, "Grid 1 Width",  64, 1, 640);
    slide_height_      = QtHelper::makeSlider(layout, "Grid 1 Height", 48, 1, 640);
    slide_width_add1_  = QtHelper::makeSlider(layout, "Grid 2 Width",  64, 1, 640);
    slide_height_add1_ = QtHelper::makeSlider(layout, "Grid 2 Height", 48, 1, 640);

}

void GridHeatMapValue::fill(QBoxLayout *layout)
{
    GridCompareValue::fill(layout);
    connect(slide_height_add1_, SIGNAL(valueChanged(int)), this, SLOT(updateState(int)));
    connect(slide_width_add1_, SIGNAL(valueChanged(int)), this, SLOT(updateState(int)));

    limit_sliders_height_.reset(new QAbstractSliderLimiter(slide_height_, slide_height_add1_));
    limit_sliders_width_.reset(new QAbstractSliderLimiter(slide_width_, slide_width_add1_));

    QPushButton *button_reset = new QPushButton("reset");
    QPushButton::connect(button_reset, SIGNAL(clicked()), this, SLOT(reset()));
    layout->addWidget(button_reset);
}

void GridHeatMapValue::updateSliderMaxima(int width, int height, int width_add1, int height_add1)
{
    GridCompare::updateSliderMaxima(width, height);
    if(private_state_ghv_->grid_height_max_add1 != height_add1) {
        private_state_ghv_->grid_height_max_add1 = height_add1;
        slide_height_add1_->setMaximum(height_add1);
    }
    if(private_state_ghv_->grid_width_max_add1 != width_add1) {
        private_state_ghv_->grid_width_max_add1 = width_add1;
        slide_width_add1_->setMaximum(width_add1);
    }

}

/// MEMENTO ------------------------------------------------------------------------------------
Memento::Ptr GridHeatMapValue::getState() const
{
    State::Ptr memento(new State);
    *memento = *boost::dynamic_pointer_cast<State>(state_);
    return memento;
}

void GridHeatMapValue::setState(Memento::Ptr memento)
{
    state_.reset(new State);
    State::Ptr s = boost::dynamic_pointer_cast<State>(memento);
    assert(s.get());
    *boost::dynamic_pointer_cast<State>(state_) = *s;
    assert(state_.get());
    private_state_gcv_ = boost::dynamic_pointer_cast<GridCompareValue::State>(state_).get();
    assert(private_state_gcv_);
    private_state_ghv_ = boost::dynamic_pointer_cast<State>(state_).get();
    assert(private_state_ghv_);

    state_mutex_.lock();
    slide_height_->setValue(private_state_ghv_->grid_height);
    slide_width_->setValue(private_state_ghv_->grid_width);
    slide_height_add1_->setValue(private_state_ghv_->grid_height_add1);
    slide_width_add1_->setValue(private_state_ghv_->grid_width_add1);
    state_mutex_.unlock();

    Q_EMIT modelChanged();
}

GridHeatMapValue::State::State() :
    GridCompareValue::State::State(),
    grid_width_add1(64),
    grid_height_add1(48),
    grid_width_max_add1(640),
    grid_height_max_add1(480)
{
}

void GridHeatMapValue::State::readYaml(const YAML::Node &node)
{
    GridCompareValue::State::readYaml(node);
    node["grid_width_add1"] >> grid_width_add1;
    node["grid_height_add1"] >> grid_height_add1;
    node["grid_width_max_add1"] >> grid_width_max_add1;
    node["grid_height_max_add1"] >> grid_height_max_add1;
}

void GridHeatMapValue::State::writeYaml(YAML::Emitter &out) const
{
    GridCompareValue::State::writeYaml(out);
    out << YAML::Key << "grid_width_add1" << YAML::Value << grid_width_add1;
    out << YAML::Key << "grid_height_add1" << YAML::Value << grid_height_add1;
    out << YAML::Key << "grid_width_max_add1" << YAML::Value << grid_width_max_add1;
    out << YAML::Key << "grid_height_max_add1" << YAML::Value << grid_height_max_add1;
}
