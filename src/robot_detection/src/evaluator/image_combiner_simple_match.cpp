/// HEADER
#include "image_combiner_simple_match.h"

/// PROJECT
#include <data/frame_io.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(robot_detection::ImageCombinerSimpleMatch, vision_evaluator::BoxedObject)

using namespace robot_detection;

cv::Mat ImageCombinerSimpleMatch::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    Frame::Ptr a = FrameIO::convert(img1, mask1);
    Frame::Ptr b = FrameIO::convert(img2, mask2);

    Frame::ExtractorFunction extractor = boost::bind(&Extractor::extract, tools->getExtractor(), _1, _2, _3, _4);
    a->extractFeatures(extractor);
    b->extractFeatures(extractor);

    std::vector<std::vector<cv::DMatch> > matches;
    tools->getMatcher()->match(a.get(), b.get(), matches);

    cv::Mat out;

    cv::Scalar matchColor = cv::Scalar::all(255);
    cv::Scalar singlePointColor = cv::Scalar(192, 64, 64);
    std::vector<std::vector<char> > mask;
    int flag = cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
    cv::drawMatches(img1, a->keypoints, img2, b->keypoints, matches, out, matchColor, singlePointColor, mask, flag);

    return out;
}
