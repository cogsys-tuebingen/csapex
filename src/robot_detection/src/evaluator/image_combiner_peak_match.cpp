/// HEADER
#include "image_combiner_peak_match.h"

/// COMPONENT
#include "option_clustering.h"

/// PROJECT
#include <data/frame_io.h>
#include <utils/opencv_utils.hpp>

/// SYSTEM
#include <boost/algorithm/string.hpp>
#include <utils/LibUtil/Stopwatch.h>
#include <utils/LibUtil/QtCvImageConverter.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(robot_detection::ImageCombinerPeakMatch, vision_evaluator::ImageCombiner)

using namespace robot_detection;

ClusteringOptions ImageCombinerPeakMatch::options;

ImageCombinerPeakMatch::ImageCombinerPeakMatch()
    : histviewer(NULL)
{
}

ImageCombinerPeakMatch::~ImageCombinerPeakMatch()
{
    if(histviewer != NULL) {
        delete histviewer;
    }
}

void ImageCombinerPeakMatch::putMultiLineText(cv::Mat combined_target, const std::string& txt)
{
    std::vector<std::string> lines;
    boost::split(lines, txt, boost::is_any_of("\n"));

    int y = 20;
    int line_height = 30;

    for(std::vector<std::string>::const_iterator it = lines.begin(); it != lines.end(); ++it) {
        cv::putText(combined_target, *it, cv::Point(20, y), CV_FONT_HERSHEY_PLAIN, 2.0, cv::Scalar::all(255), 8);
        cv::putText(combined_target, *it, cv::Point(20, y), CV_FONT_HERSHEY_PLAIN, 2.0, cv::Scalar::all(0), 2);

        y += line_height;
    }
}

void ImageCombinerPeakMatch::update_gui(QFrame* additional_holder)
{

    if(histviewer != NULL) {
        delete histviewer;
    }
    histviewer = new HistogramViewerWidget;

    QBoxLayout* layout = new QVBoxLayout;
    options.insert(layout);

    additional_holder->setLayout(layout);
}


void ImageCombinerPeakMatch::mousePressEvent(QMouseEvent* event)
{
    histviewer->mousePressEvent(event);
}

void ImageCombinerPeakMatch::mouseMoveEvent(QMouseEvent* event)
{
    histviewer->mouseMoveEvent(event);
}

void ImageCombinerPeakMatch::wheelEvent(QWheelEvent* event)
{
    histviewer->wheelEvent(event);
}

void ImageCombinerPeakMatch::keyEvent(QKeyEvent* event)
{
    histviewer->keyEvent(event);
}

void ImageCombinerPeakMatch::drawDebug(Frame::Ptr& a, Frame::Ptr& b,
                                       const cv::Mat& img1, const cv::Mat& img2, cv::Mat& out,
                                       std::vector<HoughData::Cluster> &clusters, HoughAlgorithm_Debug* h)
{
    try {
        out = cv::Mat(std::max(img1.rows, img2.rows), img1.cols + img2.cols + h->debug.cols, img1.type(), cv::Scalar::all(0));

        int offset_x = img1.cols;

        cv::Mat target_img1(out, cv::Rect(h->debug.cols, 0, img1.cols, img1.rows));
        cv::Mat target_img2(out, cv::Rect(offset_x + h->debug.cols, 0, img2.cols, img2.rows));
        cv::Mat combined_target(out, cv::Rect(h->debug.cols, 0, img2.cols + img1.cols, std::max(img1.rows, img2.rows)));

        int rest_w = h->debug.cols;
        int rest_h = out.rows - h->debug.rows;
        cv::Mat rest(out, cv::Rect(0, h->debug.rows, rest_w, rest_h));

        img1.copyTo(target_img1);
        img2.copyTo(target_img2);

        putMultiLineText(combined_target, h->log.str());

        for(unsigned i = 0; i < clusters.size(); ++i) {
            HoughData::Cluster& cluster = clusters[i];
            //            if(cluster.matches.size() < 4) {
            //                continue;
            //            }

            std::cout << "cluster " << i << ": " << cluster.matches.size() << std::endl;

            cv::Scalar c = cv::rangeColor(i, clusters.size());
            //            int radius = cluster.radius;
            cv::Point center(cluster.mean[HoughData::INDEX_X] * h->scale_factor, cluster.mean[HoughData::INDEX_Y] * h->scale_factor);
            cv::RotatedRect box(center, cv::Size(std::sqrt(cluster.variance)* h->scale_factor, std::sqrt(cluster.variance)* h->scale_factor), 0.0);
            std::cout << cluster.variance << std::endl;
            cv::ellipse(h->debug, box, c, 1, CV_AA);

            std::vector<cv::Point> points_a, points_b;
            for(unsigned j = 0; j < cluster.matches.size(); ++j) {
                const cv::DMatch& match = *cluster.matches[j];

                cv::drawKeypoint(target_img1, a->keypoints[match.queryIdx], c, cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                cv::drawKeypoint(target_img2, b->keypoints[match.trainIdx], c, cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                points_a.push_back(a->keypoints[match.queryIdx].pt);
                points_b.push_back(b->keypoints[match.trainIdx].pt);
            }

            if(points_a.size() >= 4) {

                std::vector<cv::Point> hull_a, hull_b;
                cv::convexHull(points_a, hull_a);
                cv::convexHull(points_b, hull_b);

                cv::drawConvexHull(hull_a, target_img1, c);
                cv::drawConvexHull(hull_b, target_img2, c);

                for(unsigned j = 0; j < cluster.matches.size(); ++j) {
                    const cv::DMatch& match = *cluster.matches[j];

                    cv::Point2f& target = b->keypoints[match.trainIdx].pt;

                    cv::Vec3b color_vec(c[0], c[1], c[2]);
                    double alpha = 0.5;
                    double beta = 1 - alpha;

                    cv::LineIterator it(combined_target, a->keypoints[match.queryIdx].pt, cv::Point(target.x + offset_x, target.y));
                    for(int i = 0; i < it.count; ++i, ++it) {
                        cv::Vec3b& v = *(cv::Vec3b*) *it;
                        v[0] = alpha * v[0] + beta * color_vec[0];
                        v[1] = alpha * v[1] + beta * color_vec[1];
                        v[2] = alpha * v[2] + beta * color_vec[2];
                    }
                }
            }
        }

        h->debug.copyTo(cv::Mat(out, cv::Rect(0, 0, h->debug.cols, h->debug.rows)));

        histviewer->waitForPainting(rest_w, rest_h);

        QImage img = histviewer->grabFrameBuffer();
        std::cout << img.width() << " == " << rest_w << std::endl;
        std::cout << img.height() << " == " << rest_h << std::endl;
        assert(img.width() == rest_w);
        assert(img.height() == rest_h);
        for(int y = 0; y < rest_h; ++y) {
            for(int x = 0; x < rest_w; ++x) {
                QRgb pixel = img.pixel(x, y);
                rest.at<cv::Vec3b>(y, x) = cv::Vec3b(qBlue(pixel), qGreen(pixel), qRed(pixel));
            }
        }

    } catch(cv::Exception& e) {
        WARN("cv exception: " << e.what());
    } catch(std::exception& e) {
        WARN("exception: " << e.what());
    }
}

cv::Mat ImageCombinerPeakMatch::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    Frame::Ptr a = FrameIO::convert(img1, mask1);
    Frame::Ptr b = FrameIO::convert(img2, mask2);

    Frame::ExtractorFunction extractor = boost::bind(&Extractor::extract, tools->getExtractor(), _1, _2, _3, _4);
    a->extractFeatures(extractor);
    b->extractFeatures(extractor);

    std::vector<std::vector<cv::DMatch> > matches;
    tools->getMatcher()->match(a.get(), b.get(), matches);

    Stopwatch stopwatch;

    HoughAlgorithm* h;

    if(tools->getExtractor()->hasOrientation()) {
        h = new HoughTheta(options.k, options.scaling, config.octaves, *a, *b);
    } else {
        h = new HoughNoTheta(options.k, options.scaling, config.octaves, *a, *b);
    }


    h->min_count = options.min_cluster_size;

    h->log.str(std::string());
    h->log << "setup: " << stopwatch.usElapsed() * 1e-3 << "ms\n";
    stopwatch.reset();

    std::vector<HoughData::Cluster> clusters;
    h->filter(matches, clusters);

    h->log << "---------------\noverall: " << stopwatch.usElapsed() * 1e-3 << "ms";

    histviewer->setHistogram(h->hough);

    cv::Mat out;

    HoughAlgorithm_Debug* h_dbg = dynamic_cast<HoughAlgorithm_Debug*>(h);

    if(h_dbg) {
        drawDebug(a, b, img1, img2, out, clusters, h_dbg);
    }


    return out;
}
