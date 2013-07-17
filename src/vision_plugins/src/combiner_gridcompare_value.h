#ifndef COMBINER_GRIDCOMPARE_VALUE_H
#define COMBINER_GRIDCOMPARE_VALUE_H

/// COMPONENT
#include "combiner_gridcompare.h"

namespace vision_evaluator {
/**
 * @brief The GridCompareValue class can be used to compare two images using a grid
 *        overlay. The Feature observed in this case is the mean value of values given
 *        in a grid cell.
 */
class GridCompareValue : public GridCompare
{
    Q_OBJECT

public:
    /**
     * @brief GridCompareValue default constructor.
     */
    GridCompareValue();

    /**
     * @brief combine - see base class information
     */
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);

    /**
     * @brief Update the gui state.
     * @param layout    the layout
     */
    virtual void updateGui(QBoxLayout *layout);


    /// MEMENTO
    void         setState(Memento::Ptr memento);
    Memento::Ptr getState() const;

protected Q_SLOTS:
    void updateState();

protected:
    virtual void fill(QBoxLayout *layout);
    void prepareParams(cv::Scalar &eps, cv::Vec<bool, 4> &ignore);

    QWidget                    *container_eps_slider_;
    std::vector<QDoubleSlider*> eps_sliders_;

    /// MEMENTO
    class State : public GridCompare::State {
    public:
        virtual void readYaml(const YAML::Node &node);
        virtual void writeYaml(YAML::Emitter &out) const;

    public:
        cv::Scalar          eps;
    };

    State *private_state_;
};
}
#endif // COMBINER_GRIDCOMPARE_VALUE_H
