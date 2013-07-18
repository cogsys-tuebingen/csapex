#include "combiner_gridheatmap_hist.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>
#include <QComboBox>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::GridHeatMapHist, vision_evaluator::BoxedObject)

using namespace vision_evaluator;

GridHeatMapHist::GridHeatMapHist() :
    GridCompareHist(State::Ptr(new State))
{
    private_state_ghm_ = dynamic_cast<State*>(state_.get());
    assert(private_state_ghm_);
}

void GridHeatMapHist::updateState(int value)
{
    private_state_ghm_->combo_index = combo_compare_->currentIndex();
    private_state_ghm_->grid_width  = slide_width_->value();
    private_state_ghm_->grid_height = slide_height_->value();
    private_state_ghm_->grid_width_add1  = slide_width_add1_->value();
    private_state_ghm_->grid_height_add1 = slide_height_add1_->value();
}

void GridHeatMapHist::addSliders(QBoxLayout *layout)
{
    slide_width_      = QtHelper::makeSlider(layout, "Grid 1 Width",  64, 1, 640);
    slide_height_     = QtHelper::makeSlider(layout, "Grid 1 Height", 48, 1, 640);
    slide_width_add1_  = QtHelper::makeSlider(layout, "Grid 2 Width",  64, 1, 640);
    slide_height_add1_ = QtHelper::makeSlider(layout, "Grid 2 Height", 48, 1, 640);
}

void GridHeatMapHist::fill(QBoxLayout *layout)
{
    GridCompareHist::fill(layout);
    connect(slide_height_add1_, SIGNAL(valueChanged(int)), this, SLOT(updateState(int)));
    connect(slide_width_add1_, SIGNAL(valueChanged(int)), this, SLOT(updateState(int)));
}

/// MEMENTO ------------------------------------------------------------------------------------
Memento::Ptr GridHeatMapHist::getState() const
{
    State::Ptr memento(new State);
    *memento = *boost::dynamic_pointer_cast<State>(state_);
    return memento;
}

void GridHeatMapHist::setState(Memento::Ptr memento)
{
    state_.reset(new State);
    State::Ptr s = boost::dynamic_pointer_cast<State>(memento);
    assert(s.get());
    *boost::dynamic_pointer_cast<State>(state_) = *s;
    assert(state_.get());
    private_state_gch_ = boost::dynamic_pointer_cast<GridCompareHist::State>(state_).get();
    assert(private_state_gch_);
    private_state_ghm_ = boost::dynamic_pointer_cast<State>(state_).get();
    assert(private_state_ghm_);


    slide_height_->setValue(private_state_ghm_->grid_height);
    slide_width_->setValue(private_state_ghm_->grid_width);
    slide_height_add1_->setValue(private_state_ghm_->grid_height_add1);
    slide_width_add1_->setValue(private_state_ghm_->grid_width_add1);
    combo_compare_->setCurrentIndex(private_state_ghm_->combo_index);

    Q_EMIT modelChanged();
}

GridHeatMapHist::State::State() :
    GridCompareHist::State::State(),
    grid_width_add1(48),
    grid_height_add1(64),
    grid_width_max_add1(640),
    grid_height_max_add1(480)
{
}

void GridHeatMapHist::State::readYaml(const YAML::Node &node)
{
    GridCompareHist::State::readYaml(node);
    node["grid_width_add1"] >> grid_width_add1;
    node["grid_height_add1"] >> grid_height_add1;
    node["grid_width_max_add1"] >> grid_width_max_add1;
    node["grid_height_max_add1"] >> grid_height_max_add1;
}

void GridHeatMapHist::State::writeYaml(YAML::Emitter &out) const
{
    GridCompareHist::State::writeYaml(out);
    out << YAML::Key << "grid_width_add1" << YAML::Value << grid_width_add1;
    out << YAML::Key << "grid_height_add1" << YAML::Value << grid_height_add1;
    out << YAML::Key << "grid_width_max_add1" << YAML::Value << grid_width_max_add1;
    out << YAML::Key << "grid_height_max_add1" << YAML::Value << grid_height_max_add1;
}
