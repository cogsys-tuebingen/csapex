/// HEADER
#include "detector_adapter_static.h"

/// PROJECT
#include <analyzer/analyzer.h>

/// SYSTEM
#include <utils/LibUtil/Stopwatch.h>

DetectorAdapterStatic::DetectorAdapterStatic(Detector& detector)
    : DetectorAdapter(detector), has_path(false)
{
}

DetectorAdapterStatic::~DetectorAdapterStatic()
{

}

void DetectorAdapterStatic::test_on(const std::string& path)
{
    has_path = true;
    path_ = path;
}

void DetectorAdapterStatic::test_on_callback(std::vector<Frame::Ptr >& frames)
{
    if(frames.empty()) {
        return;
    }

    analyzer.analyze(frames[frames.size() - 1]);

    frames.clear();

    usleep(2.0 * 1e6);
}

void DetectorAdapterStatic::tick(double dt)
{
    if(has_path) {
//        boost::function<void(std::vector<Frame::Ptr >&)> callback = boost::bind(&DetectorAdapterStatic::test_on_callback, this, _1);
        // TODO: raw on negative, not raw on positive
//        detector.get_context()->import_raw_directory(path_, &callback);
//        detector.get_context()->import_directory(path_, false, &callback);
    }
}

void DetectorAdapterStatic::poseDetected(const Pose& pose_camera)
{
}
