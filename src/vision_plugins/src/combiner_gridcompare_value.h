#ifndef COMBINER_GRIDCOMPARE_VALUE_H
#define COMBINER_GRIDCOMPARE_VALUE_H

/// COMPONENT
#include "combiner_gridcompare.h"

namespace csapex {
/**
 * @brief The GridCompareValue class can be used to compare two images using a grid
 *        overlay. The Feature observed in this case is the mean value of values given
 *        in a grid cell.
 */
class GridCompareValue : public GridCompare
{
    Q_OBJECT

protected:
    /// MEMENTO
    class State : public GridCompare::State {
    public:
        typedef boost::shared_ptr<State> Ptr;

        State();
        virtual void readYaml(const YAML::Node &node);
        virtual void writeYaml(YAML::Emitter &out) const;

    public:
        cv::Scalar          eps;
        cv::Vec<bool,4>     ignore;
    };

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
    virtual void updateDynamicGui(QBoxLayout *layout);


    /// MEMENTO
    void         setState(Memento::Ptr memento);
    Memento::Ptr getState() const;

protected Q_SLOTS:
    virtual void updateState(int i);

protected:
    GridCompareValue(GridCompare::State::Ptr state);

    /// fill with standard gui elements
    virtual void fill(QBoxLayout *layout);
    void prepareGrid(cv_grid::GridScalar &g, const cv::Mat &img, const cv::Mat &mask, const int width, const int height);

    /// specific preparation
    void prepareParams(cv::Scalar &eps, cv::Vec<bool, 4> &ignore);

    QWidget                    *container_eps_slider_;
    std::vector<QDoubleSlider*> eps_sliders_;

    State *private_state_gcv_;
    State state_buffer_gcv_;
};
}
#endif // COMBINER_GRIDCOMPARE_VALUE_H
