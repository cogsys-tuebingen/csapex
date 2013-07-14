#ifndef COMBINER_GRIDCOMPARE_H
#define COMBINER_GRIDCOMPARE_H

#include <vision_evaluator/image_combiner.h>
#include <utils/LibCvTools/grid.hpp>

class QSlider;

namespace vision_evaluator {
class GridCompare : public ImageCombiner
{
public:
    GridCompare();
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);

protected:
    QSlider *slide_width_;
    QSlider *slide_height_;

    virtual void fill(QBoxLayout *layout);

    template<class GridT>
    void render_grid(const GridT &g1, const GridT &g2, cv::Mat &out, std::pair<int, int> &counts)
    {
        counts.first = 0;
        counts.second = 0;
        for(int i = 0 ; i < g1.cols() ; i++) {
            for(int j = 0 ; j < g1.rows() ; j++) {
                bool cell_compare = g1(j,i) == g2(j,i);

                cv::Rect r = g1(j,i).bounding;
                if(cell_compare) {
                    cv::rectangle(out, r, cv::Scalar(0, 255, 0), 1);
                    counts.first++;
                } else {
                    cv::rectangle(out, r, cv::Scalar(0, 0, 255), 1);
                    counts.second++;
                }
            }
        }
    }


};
}
#endif // COMBINER_GRIDCOMPARE_H
