#ifndef TERRA_DECOMPOSITION_QUADTREE_H
#define TERRA_DECOMPOSITION_QUADTREE_H
#include "decomposition_quadtree.h"
#include "terra_decomposition_classifiers.hpp"

class TerraQuadtreeDecomposition : public QuadtreeDecomposition
{
public:
    TerraQuadtreeDecomposition(const cv::Mat &_image, const cv::Size &_min_region_size,
                               TerraDecomClassifier &_classifier);


};

#endif // TERRA_DECOMPOSITION_QUADTREE_H
