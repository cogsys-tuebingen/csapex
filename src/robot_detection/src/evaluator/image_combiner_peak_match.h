#ifndef IMAGE_COMBINER_PEAK_MATCH_H
#define IMAGE_COMBINER_PEAK_MATCH_H

/// COMPONENT
#include <csapex_vision/image_combiner.h>
#include "histogram_viewer_widget.h"
#include "option_clustering.h"

/// PROJECT
#include <config/reconfigurable.h>
#include <utils/hough_peak.h>

/// SYSTEM
#include <QSlider>
#include <QToolBox>
//#include <QMainWindow>

namespace robot_detection
{

class ImageCombinerPeakMatch : public csapex::ImageCombiner, public Reconfigurable
{
    Q_OBJECT

    typedef HoughPeak<true, true> HoughTheta;
    typedef HoughPeak<true, false> HoughNoTheta;

public:
    ImageCombinerPeakMatch();
    virtual ~ImageCombinerPeakMatch();

public:
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);

    void putMultiLineText(cv::Mat combined_target, const std::string& txt);

    virtual void update_gui(QFrame* additional_holder);
    virtual void insert(QBoxLayout* layout);

    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void wheelEvent(QWheelEvent* event);
    virtual void keyEvent(QKeyEvent* event);

private:
    void drawDebug(Frame::Ptr& a, Frame::Ptr& b,
                   const cv::Mat& img1, const cv::Mat& img2, cv::Mat& out,
                   std::vector<HoughData::Cluster>& clusters, HoughAlgorithm_Debug* h);

private:
//    HistogramViewerWidget* histviewer;
    static ClusteringOptions options;
//    QMainWindow* histogram_view;
};

}

#endif // IMAGE_COMBINER_PEAK_MATCH_H
