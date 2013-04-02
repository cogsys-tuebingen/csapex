#ifndef ROI_PROVIDER_H
#define ROI_PROVIDER_H

/// COMPONENT
#include "roi.h"

/// PROJECT
#include <data/frame.h>

/**
 * @brief The RoiProvider class is a base class for classes that produces regions of interest
 */
class RoiProvider
{
public:
    /**
     * @brief RoiProvider
     */
    RoiProvider();

    /**
     * @brief ~RoiProvider
     */
    virtual ~RoiProvider();

    /**
     * @brief newFrame signal that a new frame is analyzed
     * @param current_frame
     */
    virtual void newFrame(Frame* current_frame);

    /**
     * @brief addRois integrate all produced rois into the cluster
     * @param rois the list of rois to update
     */
    virtual void addRois(std::vector<Roi>& rois) = 0;

    /**
     * @brief roiContainsTarget
     * @param roi
     */
    virtual void roiContainsTarget(const Roi& roi) {}

    /**
     * @brief setFrameSize setter for the maximum roi area
     * @param size
     */
    void setFrameSize(const cv::Rect& size);

protected:
    cv::Rect frame_size_;
    Frame* current_frame_;
};

#endif // ROI_PROVIDER_H
