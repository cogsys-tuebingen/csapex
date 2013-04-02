/// HEADER
#include "detector_adapter.h"

/// PROJECT
#include <analyzer/detector.h>
#include <data/matchable_pose.h>

DetectorAdapter::DetectorAdapter(Detector& detector)
    : AnalyzerAdapter(detector), detector(detector)
{
    detector.pose_detected.connect(boost::bind(&DetectorAdapter::poseDetected, this, _1));
}

DetectorAdapter::~DetectorAdapter()
{
}
