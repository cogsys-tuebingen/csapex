#ifndef COMBINER_GRIDCOMPARE_HIST_H
#define COMBINER_GRIDCOMPARE_HIST_H

/// COMPONENT
#include "combiner_gridcompare.h"

class QComboBox;

namespace vision_evaluator {
/**
 * @brief The GridCompareHist class can be used to compare two images using a grid
 *        overlay. The Feature observed in this case is the histogram calculated on the
 *        gridcells.
 */
class GridCompareHist : public GridCompare
{
public:
    /**
     * @brief GridCompareHist default constructor.
     */
    GridCompareHist();
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
    /// internal typedefs
    typedef std::pair<QSlider*, QDoubleSlider*> HistSliderPair;
    typedef std::vector<HistSliderPair>         HistPreferences;
    typedef std::pair<int, int>                 intPair;

    QWidget                    *container_hist_sliders_;
    QComboBox                  *combo_compare_;
    HistPreferences            hist_sliders_;
    std::map<int, int>         index_to_compare_;

    /// fill with specific gui elements
    virtual void fill(QBoxLayout *layout);
    void insertSliders(QSlider *bins, QDoubleSlider *eps);
    void prepareHistParams(cv::Mat &bins, cv::Mat &ranges, cv::Scalar &eps);

    /// MEMENTO
    class State : public Memento {
    public:
        void readYaml(const YAML::Node &node)
        {
            node["channel_count"] >> channel_count;
            node["grid_width"] >> grid_width;
            node["grid_height"] >> grid_height;
            node["compare"] >> combo_index;

            const YAML::Node &_bins = node["bins"];
            for(YAML::Iterator it = _bins.begin() ; it != _bins.end() ; it++) {
                int bin_val;
                *it >> bin_val;
                bins.push_back(bin_val);
            }

            const YAML::Node &_eps = node["eps"];
            for(YAML::Iterator it = _eps.begin() ; it != _eps.end() ; it++) {
                double eps_val;
                *it >> eps_val;
                eps.push_back(eps_val);
            }
            restored = true;
        }

        void writeYaml(YAML::Emitter &out) const
        {
            out << YAML::Key << "channel_count" << YAML::Value << channel_count;
            out << YAML::Key << "grid_width" << YAML::Value << grid_width;
            out << YAML::Key << "grid_height" << YAML::Value << grid_height;
            out << YAML::Key << "compare" << YAML::Value << combo_index;

            out << YAML::Key << "bins" << YAML::Value << YAML::BeginSeq;
            for(std::vector<int>::const_iterator it = bins.begin() ; it != bins.end() ; it++) {
                out << *it;
            }
            out << YAML::EndSeq;

            out << YAML::Key << "eps" << YAML::Value << YAML::BeginSeq;
            for(std::vector<double>::const_iterator it = eps.begin() ; it != eps.end() ; it++) {
                out << *it;
            }
            out << YAML::EndSeq;

        }

    public:
        int                 combo_index;
        int                 channel_count;
        int                 grid_width;
        int                 grid_height;
        std::vector<int>    bins;
        std::vector<double> eps;
        bool                restored;

    };

    State state_;
};
}
#endif // COMBINER_GRIDCOMPARE_HIST_H
