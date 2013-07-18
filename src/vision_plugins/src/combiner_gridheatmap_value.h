#ifndef COMBINER_GRIDHEATMAP_VALUE_H
#define COMBINER_GRIDHEATMAP_VALUE_H

#include "combiner_gridcompare_value.h"
#include <vision_evaluator/qwrapper.h>

namespace vision_evaluator {
class GridHeatMapValue : public GridCompareValue
{

    Q_OBJECT

public:
    GridHeatMapValue();

    /// TODO : override combine !

    /// MEMENTO
    void setState(Memento::Ptr memento);
    Memento::Ptr getState() const;

protected Q_SLOTS:
    virtual void updateState(int value);

protected:
    virtual void addSliders(QBoxLayout *layout);

    QSlider *slide_width_add1_;
    QSlider *slide_height_add1_;

//    boost::shared_ptr<QLimiterSlider> limit_sliders_width_;
//    boost::shared_ptr<QLimiterSlider> limit_sliders_height_;

    /// fill with standard gui elements
    virtual void fill(QBoxLayout *layout);

    /// MEMENTO
    class State : public GridCompareValue::State {
    public:
        State();

        typedef boost::shared_ptr<State> Ptr;
        virtual void readYaml(const YAML::Node &node);
        virtual void writeYaml(YAML::Emitter &out) const;

    public:
        int grid_width_add1;
        int grid_height_add1;
        int grid_width_max_add1;
        int grid_height_max_add1;
    };

    State *private_state_ghv_;

};
}
#endif // COMBINER_GRIDHEATMAP_VALUE_H
