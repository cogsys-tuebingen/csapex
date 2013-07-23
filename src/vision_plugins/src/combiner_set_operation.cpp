#include "combiner_set_operation.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QComboBox>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::SetOperation, vision_evaluator::BoxedObject)

using namespace vision_evaluator;

SetOperation::SetOperation() :
    ImageCombiner()
{
}

cv::Mat SetOperation::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    if(!img1.empty() && !img2.empty()) {
        /// PREPARE
        if(img1.channels() != 1 || img2.channels() != 1)
            throw std::runtime_error("No Single Channel!");
        if(img1.rows != img2.rows || img1.cols != img2.cols)
            throw std::runtime_error("Dimension is not matching!");
    }

    if (state_.operation_index == 0)
        return ~img1;
    if (state_.operation_index == 1)
        return img1 & img2;
    if (state_.operation_index == 2)
        return img1 | img2;
}

void SetOperation::updateDynamicGui(QBoxLayout *layout)
{
    QVBoxLayout *internal_layout;
    internal_layout = new QVBoxLayout;
}

Memento::Ptr SetOperation::getState() const
{
    boost::shared_ptr<State> memento(new State);
    *memento = state_;

    return memento;
}

void SetOperation::setState(Memento::Ptr memento)
{
    boost::shared_ptr<State> m = boost::dynamic_pointer_cast<State> (memento);
    assert(m.get());

    state_ = *m;
    combo_compare_->setCurrentIndex(state_.operation_index);
}

void SetOperation::updateState(int i)
{
    state_.operation_index = combo_compare_->currentIndex();

/*    private_state_gcv_->grid_width  = slide_width_->value();
    private_state_gcv_->grid_height = slide_height_->value();
    prepareParams(private_state_gcv_->eps, private_state_gcv_->ignore);
*/}

void SetOperation::fill(QBoxLayout *layout)
{
    ImageCombiner::fill(layout);

    combo_compare_ = new QComboBox();
    combo_compare_->addItem("Complement");
    combo_compare_->addItem("Intersection");
    combo_compare_->addItem("Union");

    layout->addWidget(combo_compare_);
    state_.operation_index = combo_compare_->currentIndex();

    connect(combo_compare_, SIGNAL(currentIndexChanged(int)), this, SLOT(updateState(int)));
}

void SetOperation::State::readYaml(const YAML::Node &node)
{
    node["operation_index"] >> operation_index;
}

void SetOperation::State::writeYaml(YAML::Emitter &out) const
{
    out << YAML::Key << "operation_index" << YAML::Value << operation_index;
}
