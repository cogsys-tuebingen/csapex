#include "terra_decomposition_quadtree.h"

TerraQuadtreeDecomposition::TerraQuadtreeDecomposition(const cv::Mat &_image, const cv::Size &_min_region_size,
                                                       TerraDecomClassifier &_classifier) :
    QuadtreeDecomposition(_image, _min_region_size, _classifier)
{
}
