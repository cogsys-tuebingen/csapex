#ifndef SCREEN_SPACE_ROI_TRACKER_H
#define SCREEN_SPACE_ROI_TRACKER_H

/// COMPONENT
#include "roi_provider.h"

/**
 * @brief The ScreenSpaceRoiTracker class tracks rectangular regions in screen space
 */
class ScreenSpaceRoiTracker : public RoiProvider
{
public:
    /**
     * @brief ScreenSpaceRoiTracker
     */
    ScreenSpaceRoiTracker();

    /**
     * @brief addRois integrate all produced rois into the cluster
     * @param rois the list of rois to update
     */
    void addRois(std::vector<Roi>& rois);

    /**
     * @brief roiContainsTarget
     * @param roi
     */
    void roiContainsTarget(const Roi& roi);

    /**
     * @brief newFrame signal that a new frame is analyzed
     * @param current_frame
     */
    virtual void newFrame(Frame* current_frame);

    /**
     * @brief isTracking
     * @param roi
     * @param covered_by: Output: the roi that covers 'roi'
     * @return <b>true</b>, iff roi is already tracked
     */
    bool isTracking(const Roi& roi, Roi *&covered_by);

private:
    std::vector<Roi> tracked_;
};

#endif // SCREEN_SPACE_ROI_TRACKER_H
