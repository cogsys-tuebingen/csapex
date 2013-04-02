#ifndef DETECTOR_NODE_H
#define DETECTOR_NODE_H

/// COMPONENT
#include "analyzer_adapter.h"

/// FORWARD DECLARATION
class Detector;
class Pose;

/**
 * @brief The DetectorAdapter class is a base class for Detector adapters
 */
class DetectorAdapter : public AnalyzerAdapter
{
protected:
    /**
     * @brief DetectorAdapter
     * @param initial config
     * @param detector The detector to wrap
     * @throws if something went wrong
     */
    DetectorAdapter(Detector& detector);

public:
    /**
     * @brief ~DetectorAdapter
     */
    virtual ~DetectorAdapter();


protected:
    virtual void poseDetected(const Pose& pose_camera) = 0;

protected:
    Detector& detector;
};

#endif // DETECTOR_SINGLE_H
