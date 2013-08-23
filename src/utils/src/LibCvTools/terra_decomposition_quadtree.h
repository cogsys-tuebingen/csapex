#ifndef TERRA_DECOMPOSITION_QUADTREE_H
#define TERRA_DECOMPOSITION_QUADTREE_H
#include "decomposition_quadtree.h"
#include "roi.hpp"

class TerraQuadtreeDecomposition : public QuadtreeDecomposition
{
public:
    typedef boost::shared_ptr<TerraQuadtreeDecomposition> Ptr;
    typedef QuadtreeDecomposition::CVQt                   CVQt;
    typedef QuadtreeDecomposition::CVQtNodesListPtr          CVQtNodesListPtr;
    typedef std::pair<CVQt*, cv_roi::TerraID>             ClassificationEntry;

    TerraQuadtreeDecomposition(const cv::Mat &_image, const cv::Size &_min_region_size,
                               TerraDecomClassifierFeature *_classifier);

    virtual bool iterate();

    void regions(std::vector<cv_roi::TerraROI> &regions);

protected:
    std::map<CVQt*, cv_roi::TerraID>  classifications_;

    virtual void                process_active_nodes();
};

#endif // TERRA_DECOMPOSITION_QUADTREE_H
