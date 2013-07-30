#ifndef COMBINER_GRIDCOMPARE_H
#define COMBINER_GRIDCOMPARE_H

/// PROJECT
#include <vision_evaluator/image_combiner.h>
#include <utils/LibCvTools/grid.hpp>
#include <utils/LibCvTools/grid_attributes.hpp>
#include <utils/LibCvTools/grid_compute.hpp>
#include <csapex/qt_helper.hpp>

/// SYSTEM
#include <QMutex>

class QSlider;

namespace csapex {
class GridCompare : public ImageCombiner
{
protected:
    /// MEMENTO
    class State : public Memento {
    public:
        State();

        virtual void readYaml(const YAML::Node &node);
        virtual void writeYaml(YAML::Emitter &out) const;

        typedef boost::shared_ptr<State> Ptr;

    public:
        int                 channel_count;
        int                 grid_width;
        int                 grid_height;
        int                 grid_width_max;
        int                 grid_height_max;
    };

public:
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2) = 0;
    virtual void updateDynamicGui(QBoxLayout *layout);

    /// MEMENTO
    virtual void         setState(Memento::Ptr memento) = 0;
    virtual Memento::Ptr getState() const = 0;

protected:
    GridCompare(State::Ptr state);

    QSlider *slide_width_;
    QSlider *slide_height_;

    virtual void fill(QBoxLayout *layout);
    virtual void addSliders(QBoxLayout *layout);
    virtual void updateSliderMaxima(int width, int height);

    QMutex  state_mutex_;
    mutable State::Ptr state_;

};
}
#endif // COMBINER_GRIDCOMPARE_H
