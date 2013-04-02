#ifndef NTH_FULL_FRAME_ROI_H
#define NTH_FULL_FRAME_ROI_H

/// COMPONENT
#include "roi_provider.h"

/**
 * @brief The NthFullFrameRoi class skips (N-1) frames and analyzes the Nth frame completely
 */
class NthFullFrameRoi : public RoiProvider
{
public:
    /**
     * @brief NthFullFrameRoi
     * @param n number of frames in each cycle
     */
    NthFullFrameRoi(unsigned n);

    /**
     * @brief addRois integrate all produced rois into the cluster
     * @param rois the list of rois to update
     */
    void addRois(std::vector<Roi>& rois);

private:
    unsigned n;
    unsigned counter_;
};

#endif // NTH_FULL_FRAME_ROI_H
