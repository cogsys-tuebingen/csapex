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
protected:
    /// internal typedefs
    typedef std::pair<QSlider*, QDoubleSlider*> HistSliderPair;
    typedef std::vector<HistSliderPair>         HistPreferences;
    typedef std::pair<int, int>                 intPair;

    int                        channel_count_;
    QBoxLayout *layout_;
    QWidget                    *container_hist_sliders_;
    QComboBox                  *combo_compare_;
    HistPreferences            hist_sliders_;
    std::map<int, int>         index_to_compare_;
    std::map<int, int>         compare_to_index_;


    /// fill with specific gui elements
    virtual void fill(QBoxLayout *layout);
    void updateSliders();
    void insertSliders(QSlider *bins, QDoubleSlider *eps);
    void prepareHistParams(cv::Mat &bins, cv::Mat &ranges, cv::Scalar &eps);
};
}
#endif // COMBINER_GRIDCOMPARE_HIST_H
