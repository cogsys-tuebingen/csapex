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
protected:
    virtual void fill(QBoxLayout *layout);
    void prepareParams(cv::Scalar &eps, cv::Vec<bool, 4> &ignore);

    QWidget                    *container_eps_slider_;
    std::vector<QDoubleSlider*> eps_sliders_;

    /// MEMENTO
    class State : public Memento {
    public:
        void readYaml(const YAML::Node &node)
        {
            node["channel_count"] >> channel_count;
            node["grid_width"] >> grid_width;
            node["grid_height"] >> grid_height;
            const YAML::Node &_eps = node["eps"];
            int i = 0;
            for(YAML::Iterator it = _eps.begin() ; it != _eps.end() ; it++, i++) {
                *it >> eps[i];
            }

            restored = true;
        }

        void writeYaml(YAML::Emitter &out) const
        {
            out << YAML::Key << "channel_count" << YAML::Value << channel_count;
            out << YAML::Key << "grid_width" << YAML::Value << grid_width;
            out << YAML::Key << "grid_height" << YAML::Value << grid_height;
            out << YAML::Key << "eps" << YAML::Value << YAML::BeginSeq;
            for(int i = 0 ; i < 4 ; i++) {
                out << eps[i];
            }
            out << YAML::EndSeq;

        }

    public:
        int                 channel_count;
        cv::Scalar          eps;
        bool                restored;
        int                 grid_width;
        int                 grid_height;

    };

    State state_;

};
}
#endif // COMBINER_GRIDCOMPARE_VALUE_H
