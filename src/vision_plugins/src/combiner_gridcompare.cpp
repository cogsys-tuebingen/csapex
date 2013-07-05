#include "combiner_gridcompare.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>

/// PROJECT
#include <utils/LibCvTools/grid_attributes.hpp>
#include <utils/LibCvTools/grid_compute.hpp>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::GridCompare, vision_evaluator::ImageCombiner)


using namespace vision_evaluator;
using namespace cv_grid;

GridCompare::GridCompare()
{
}

cv::Mat GridCompare::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    if(!img1.empty() && !img2.empty()) {
        GridHist g1, g2;
        AttrHistogram::Params p;
        p.bins = (cv::Mat_<int>(3,1) << 32, 32, 32);
        p.eps  = cv::Scalar(97,97,97);
        p.method = CV_COMP_CHISQR;
        p.ranges = (cv::Mat_<float>(6,1) << 0.f, 256.f,0.f, 256.f,0.f, 256.f);
        cv::Mat img1_;
        cv::Mat img2_;

        cv::cvtColor(img1, img1_, CV_BGR2YUV);
        cv::cvtColor(img2, img2_, CV_BGR2YUV);

        cv_grid::prepare_grid<AttrHistogram>(g1, img1_, 56, 72, p);
        cv_grid::prepare_grid<AttrHistogram>(g2, img2_, 56, 72, p);


        std::cout << img1.rows << " " << img1.cols << std::endl;

        cv::Mat out(img1.rows, img1.cols, CV_8UC3, cv::Scalar(0,0,0));
        std::pair<int, int> counts;
        render_grid(g1, g2, out, counts);

        std::cout << "COUNTS +" << counts.first << " -" << counts.second << std::endl;

        return out;
    }
    return cv::Mat();
}

void GridCompare::fill(QBoxLayout *layout)
{
    ImageCombiner::fill(layout);
}
