#ifndef TERRA_DECOMPOSITION_QUADTREE_H
#define TERRA_DECOMPOSITION_QUADTREE_H
#include "decomposition_quadtree.h"
#include "terra_decomposition_classifiers.hpp"

class TerraQuadtreeDecomposition : public QuadtreeDecomposition
{
public:
    struct TerraID {
        float prob;
        int   id;
    };

    struct TerraRect {
        cv::Rect rect;
        TerraID  id;
    };

    typedef QuadtreeDecomposition::CVQt          CVQt;
    typedef QuadtreeDecomposition::CVQtNodesList CVQtNodesList;
    typedef std::pair<CVQt*, TerraID>             ClassificationEntry;

    TerraQuadtreeDecomposition(const cv::Mat &_image, const cv::Size &_min_region_size,
                               TerraDecomClassifier &_classifier);

    virtual bool iterate();

    void regions(std::vector<TerraRect> &regions);

protected:
    std::map<CVQt*, TerraID>     classifications_;

    virtual void                process_active_nodes();
};

#endif // TERRA_DECOMPOSITION_QUADTREE_H
