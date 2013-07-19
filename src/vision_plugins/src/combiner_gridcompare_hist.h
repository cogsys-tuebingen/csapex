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
        int                 combo_index;
        std::vector<int>    bins;
        std::vector<double> eps;
    };

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
    virtual void updateDynamicGui(QBoxLayout *layout);

    /// MEMENTO
    void         setState(Memento::Ptr memento);
    Memento::Ptr getState() const;

protected Q_SLOTS:
    virtual void updateState(int value);

protected:
    GridCompareHist(GridCompare::State::Ptr state);

    /// internal typedefs
    typedef std::pair<QSlider*, QDoubleSlider*> HistSliderPair;
    typedef std::vector<HistSliderPair>         HistPreferences;
    typedef std::pair<int, int>                 intPair;

    QWidget                    *container_hist_sliders_;
    QComboBox                  *combo_compare_;
    HistPreferences            hist_sliders_;
    std::map<int, int>         index_to_compare_;

    /// fill with standard gui elements
    virtual void fill(QBoxLayout *layout);
    void prepareGrids(cv_grid::GridHist &g1, cv_grid::GridHist &g2, const cv::Mat &img1, const cv::Mat &img2, const cv::Mat &mask1, const cv::Mat &mask2);

    /// dynamic gui
    void addHistSliders(QSlider *bins, QDoubleSlider *eps);
    void prepareHistParams(cv::Mat &bins, cv::Mat &ranges, cv::Scalar &eps);

    State *private_state_gch_;

};
}
#endif // COMBINER_GRIDCOMPARE_HIST_H
