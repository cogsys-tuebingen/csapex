#ifndef IMAGE_COMBINER_CLUSTER_MATCH_H
#define IMAGE_COMBINER_CLUSTER_MATCH_H

/// COMPONENT
#include "image_combiner.h"

/// PROJECT
#include <config/reconfigurable.h>

/// SYSTEM
#include <utils/LibClustering/Clustering.h>
#include <QSlider>

namespace lib_clustering
{

typedef std::pair<std::vector<int>, std::pair<unsigned, cv::DMatch*> > PointT;
typedef std::vector<PointT> PointList;
typedef KMeans<4, PlusPlusInitialization, EuclideanDistance, SparseNonUnique, PointList, cv::DMatch > KMeansAlgorithm;

}

class ImageCombinerClusterMatch : public ImageCombiner, public Reconfigurable
{
    Q_OBJECT

public:
    ImageCombinerClusterMatch(const std::string& label);

public:
    static ImageCombiner::TypePtr createInstance(CONSTRUCTOR_MODE mode);

public:
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);

};

#endif // IMAGE_COMBINER_CLUSTER_MATCH_H
