#ifndef COMBINER_GRIDHEATMAP_VALUE_H
#define COMBINER_GRIDHEATMAP_VALUE_H

/// COMPONENT
#include "combiner_gridcompare_value.h"
#include <csapex/qsignal_bridges.h>

namespace csapex {
class GridHeatMapValue : public GridCompareValue
{

    Q_OBJECT

public:
    GridHeatMapValue();

    /**
     * @brief combine - see base class information
     */
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);

    /// MEMENTO
    void         setState(Memento::Ptr memento);
    Memento::Ptr getState() const;

protected Q_SLOTS:
    virtual void updateState(int value);
    virtual void reset();

protected:
    enum RunState {RESET, RUNNING, BUFFERING};

    virtual void addSliders(QBoxLayout *layout);

    QSlider *slide_width_add1_;
    QSlider *slide_height_add1_;
    RunState run_state_;
    cv::Mat  buffered_image_;

    QSignalBridges::QAbstractSliderLimiter::Ptr limit_sliders_width_;
    QSignalBridges::QAbstractSliderLimiter::Ptr limit_sliders_height_;

    /// fill with standard gui elements
    virtual void fill(QBoxLayout *layout);
    virtual void updateSliderMaxima(int width, int height, int width1, int height1);

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
    State state_buffer_ghv_;

};
}
#endif // COMBINER_GRIDHEATMAP_VALUE_H
