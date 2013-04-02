/// HEADER
#include "viola_jones_roi.h"

/// PROJECT
#include <common/global.hpp>
#include <data/painter.h>
#include <utils/rectangle_cluster.h>

/// SYSTEM
#include <CascadeDetector.h>

ViolaJonesRoi::ViolaJonesRoi()
    : vj_detector(NULL), image_scanner(NULL)
{
    std::string filename = std::string(getenv("RABOT")) + "/Config/RobotDetection/classifier.detector";

    if(!filename.empty()) {
        std::ifstream in(filename.c_str());
        if(!in.is_open()) {
            INFO("Error Viola Jones Classifier from file: " << filename);
        } else {
            vj_detector = CascadeDetector::load(&in, false, false);

            image_scanner = new ImageScanner(vj_detector,
                                             7.5, // minscale
                                             55.0, // maxscale
                                             1.414213562);

            INFO("loading Viola Jones Classifier succeeded (" << filename << ")");
        }
    }
}

ViolaJonesRoi::~ViolaJonesRoi()
{
    if(image_scanner != NULL) {
        delete image_scanner;
    }
}

void ViolaJonesRoi::addRois(std::vector<Roi> &rois)
{
    // @todo: only convert once
    cv::Mat gray;
    cv::cvtColor(current_frame_->getImage(), gray, CV_RGB2GRAY);
    QImage img(gray.data, gray.cols, gray.rows, QImage::Format_Indexed8);
    image_scanner->setImage(&img);
    utilities::rectangle rect;
    std::vector<utilities::rectangle> rectangleList;

    RectangleCluster cluster;

    while(image_scanner->scanNext(&rect)) {
        rectangleList.push_back(rect);

        cv::Rect rec(rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top);
        Painter(current_frame_).drawRectangle(rec, cv::Scalar(255, 0, 0), 1);

        cluster.integrate(rec);
    }

    for(std::vector<cv::Rect>::const_iterator it = cluster.begin(); it != cluster.end(); ++it) {
        rois.push_back(Roi(*it));
    }

}
