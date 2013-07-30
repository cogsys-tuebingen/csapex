#ifndef IMAGE_COMBINER_CLUSTER_MATCH_H
#define IMAGE_COMBINER_CLUSTER_MATCH_H

/// COMPONENT
#include <vision_evaluator/image_combiner.h>
#include "option_clustering.h"

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

namespace robot_detection
{

class ImageCombinerClusterMatch : public csapex::ImageCombiner, public Reconfigurable
{
    Q_OBJECT

public:
    virtual cv::Mat combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2);
    virtual void update_gui(QFrame* additional_holder);
    virtual void insert(QBoxLayout* layout);

private:
    static ClusteringOptions options;
};

}

#endif // IMAGE_COMBINER_CLUSTER_MATCH_H
