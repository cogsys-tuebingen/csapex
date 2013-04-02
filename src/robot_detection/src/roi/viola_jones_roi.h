#ifndef VIOLA_JONES_ROI_H
#define VIOLA_JONES_ROI_H

/// COMPONENT
#include "roi_provider.h"

/// FORWARD DECLARATION
class ImageScanner;
class CascadeDetector;

/**
 * @brief The ViolaJonesRoi class uses a VJ cascade filter to find regions, where a robot might be
 */
class ViolaJonesRoi : public RoiProvider
{
public:
    /**
     * @brief ViolaJonesRoi
     */
    ViolaJonesRoi();

    /**
     * @brief ~ViolaJonesRoi
     */
    virtual ~ViolaJonesRoi();

    /**
     * @brief addRois integrate all produced rois into the cluster
     * @param rois the list of rois to update
     */
    void addRois(std::vector<Roi>& rois);

private:
    CascadeDetector* vj_detector;
    ImageScanner* image_scanner;
};

#endif // VIOLA_JONES_ROI_H
