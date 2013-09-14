/// HEADER
#include "grusig_descriptor.h"

/// SYSTEM
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/stats.hpp>

using namespace cv;
namespace ba = boost::accumulators;

GRUSIG::GRUSIG(int dimension)
    : cols_(3), dim_(dimension)
{
    assert(dim_ > 0);
}
GRUSIG::~GRUSIG()
{}

/** returns the descriptor length in bytes */
int GRUSIG::descriptorSize() const
{
    return cols_ * sizeof(float);
}

/** returns the descriptor type */
int GRUSIG::descriptorType() const
{
    return CV_32FC1;
}

void GRUSIG::computeImpl( const Mat& image, std::vector<KeyPoint>& keypoints, Mat& descriptors) const
{
    assert(image.type() == CV_8UC3);

    std::vector<KeyPoint>::iterator it;
    std::vector<KeyPoint>::iterator end = keypoints.end();
    for(it = keypoints.begin(); it != end;) {
        const KeyPoint& kp = *it;
        if(kp.pt.x <= dim_ || kp.pt.x >= image.cols - dim_ - 1 ||
                kp.pt.y <= dim_ || kp.pt.y >= image.rows - dim_ - 1) {
            it = keypoints.erase(it);
        } else {
            ++it;
        }
    }

    size_t n = keypoints.size();

    descriptors.create(n, cols_, descriptorType());

    for(size_t i = 0; i < n; ++i) {
        computeRow(image, descriptors.row(i), keypoints[i]);
    }
}

void GRUSIG::computeRow(const Mat& image, cv::Mat out, KeyPoint& kp) const
{
    assert(out.type() == CV_32FC1);

    unsigned fromx = kp.pt.x - dim_;
    unsigned fromy = kp.pt.y - dim_;
    unsigned tox = kp.pt.x + dim_;
    unsigned toy = kp.pt.y + dim_;

    ba::accumulator_set<double, ba::stats<ba::tag::variance> > r;
    ba::accumulator_set<double, ba::stats<ba::tag::variance> > g;
    ba::accumulator_set<double, ba::stats<ba::tag::variance> > b;

    for(unsigned dy = fromy; dy <= toy; ++dy) {
        for(unsigned dx = fromx; dx <= tox; ++dx) {
            const cv::Vec3b& val = image.at<cv::Vec3b>(dy, dx);

            b(val[0]);
            g(val[1]);
            r(val[20]);
        }
    }

    int i = -1;
    out.at<float>(0, ++i) = ba::mean(r);
    out.at<float>(0, ++i) = ba::mean(g);
    out.at<float>(0, ++i) = ba::mean(b);
    out.at<float>(0, ++i) = ba::variance(r);
    out.at<float>(0, ++i) = ba::variance(g);
    out.at<float>(0, ++i) = ba::variance(b);
}
