/// HEADER
#include "image_combiner_cluster_match.h"

/// COMPONENT
#include "option_clustering.h"

/// PROJECT
#include <data/frame_io.h>

REGISTER_IMAGE_COMBINER(ImageCombinerClusterMatch)

using namespace lib_clustering;

ImageCombinerClusterMatch::ImageCombinerClusterMatch(const std::string& label)
    : ImageCombiner(label)
{
}


// register the custom point type for the generic k means implementation
namespace lib_clustering
{

template <>
struct AccessTraits<int, PointList> {
    static int index(const PointT& in, unsigned index) {
        return in.first[index];
    }
    static int value(const PointT& in) {
        return in.second.first;
    }
    template <class VectorT, class IndexProvider, class Type>
    static VectorT readVector(const PointT& in, Type& val) {
        VectorT vector(val);
        copy(vector, in);
        assert(vector.data != NULL);
        return vector;
    }
    template <class VectorT>
    static VectorT readEmptyVector(const PointT& in) {
        VectorT vector;
        copy(vector, in);
        assert(vector.data != NULL);
        return vector;
    }

    template <class VectorT>
    static void copy(VectorT& vector, const PointT& in) {
        for(unsigned dim = 0; dim < VectorT::Dimension; ++dim) {
            vector[dim] = AccessTraits::index(in, dim);
        }
        vector.data = in.second.second;
    }
};
}

class Hough
{

public:
    Hough(int cluster_count, int octaves, Frame::Ptr a, Frame::Ptr b)
        : k(cluster_count), algo(k), octaves(octaves), a(a), b(b) {
        padding = 1;
        scaling = 16;

        dim_theta = 360 / 30;
        dim_sigma = 2 * octaves;
        dim_tx = 2 * std::max(a->getWidth(), b->getWidth()) / scaling;
        dim_ty = 2 * std::max(a->getHeight(), b->getHeight()) / scaling;

        dim1 = dim_theta;
        dim2 = dim1 * dim_sigma;
        dim3 = dim2 * dim_tx;

        hough = new unsigned[(dim_theta+2*padding) * (dim_sigma+2*padding) * (dim_tx+2*padding) * (dim_ty+2*padding)];
    }

    ~Hough() {
        delete[] hough;
    }

    void filter(std::vector<std::vector<cv::DMatch> >& matches,
                std::vector<KMeansAlgorithm::ClusterT>& clusters) {

        debug = cv::Mat(dim_ty + 2*padding, dim_tx + 2*padding, CV_8UC3, cv::Scalar::all(255));

        for(std::vector<std::vector<cv::DMatch> >::iterator it = matches.begin(); it != matches.end(); ++it) {
            if(it->size() == 0) {
                continue;
            }
            if(it[0].size() == 0) {
                continue;
            }

            cv::DMatch& match = it[0][0];

            assert((unsigned) match.queryIdx < a->keypoints.size());
            assert((unsigned) match.trainIdx < b->keypoints.size());

            const cv::KeyPoint& ka = a->keypoints[match.queryIdx];
            const cv::KeyPoint& kb = b->keypoints[match.trainIdx];

            int dsigma = ka.octave - kb.octave;
            float dtheta = Angle(ka.angle - kb.angle).toDegrees();
            int tx = (ka.pt.x - kb.pt.x) / scaling;
            int ty = (ka.pt.y - kb.pt.y) / scaling;

            int isigma = octaves + dsigma;
            int itheta = (180 + dtheta) / 360 * dim_theta;
            int itx = dim_tx / 2 + tx;
            int ity = dim_ty / 2 + ty;

            assert(0 <= isigma);
            assert(isigma < dim_sigma);
            assert(0 <= itheta);
            assert(itheta < dim_theta);

            if(itx < 0 || itx >= dim_tx || ity < 0 || ity >= dim_ty) {
                it->clear();

            } else {
                // vote

//                PointT pt(std::vector<int>(), std::pair<unsigned, cv::DMatch*>(1, &match));
//                pt.first.push_back(itheta);
//                pt.first.push_back(isigma);
//                pt.first.push_back(itx);
//                pt.first.push_back(ity);
//                candidates.push_back(pt);

                //  debug.at<cv::Vec3b>(padding + ity, padding + itx) -= cv::Vec3b::all(64);

                for(int s = -padding; s <= padding; ++s) {
                    for(int t = -padding; t <= padding; ++t) {
                        for(int u = -padding; u <= padding; ++u) {
                            for(int v = -padding; v <= padding; ++v) {
                                vote(itheta+s, isigma+t, itx+u, ity+v);
                            }
                        }
                    }
                }
            }
        }

        if((int) candidates.size() < k) {
            WARN("cannot cluster, not enough matches");
            return;
        }

        algo.find(candidates, limits, clusters);
    }

    inline void vote(int itheta, int isigma, int itx, int ity) {
        assert(0 <= ity + padding);
        assert(ity < debug.rows + padding);
        assert(0 <= itx + padding);
        assert(itx < debug.cols + padding);
        hough[(itheta + padding) + (isigma + padding) * dim1 + (itx + padding) * dim2 + (ity + padding) * dim3]++;
        debug.at<cv::Vec3b>(padding + ity, padding + itx) -= cv::Vec3b::all(5);
    }

private:
    int k;
    KMeansAlgorithm algo;

    int padding;
    int scaling;

    int dim_theta;
    int dim_sigma;
    int dim_tx;
    int dim_ty;

    int dim1;
    int dim2;
    int dim3;

    int octaves;
    Frame::Ptr a;
    Frame::Ptr b;

    KMeansAlgorithm::InputDataTypeT candidates;
    KMeansAlgorithm::LimitPairList limits;

    unsigned* hough;

public:
    cv::Mat debug;
};


namespace
{
cv::Scalar color(int i, int k)
{
    cv::Mat col(1,1,CV_8UC3, cv::Scalar((255.0 * i) / k, 255, 255));
    cv::cvtColor(col, col, CV_HSV2BGR);
    cv::Vec3b s = col.at<cv::Vec3b>(0,0);
    return cv::Scalar(s[0], s[1], s[2]);
}
}

namespace cv
{
const int draw_shift_bits = 4;
const int draw_multiplier = 1 << draw_shift_bits;

static inline void drawKeypoint(Mat& img, const KeyPoint& p, const Scalar& color, int flags)
{
    CV_Assert(!img.empty());
    Point center(cvRound(p.pt.x * draw_multiplier), cvRound(p.pt.y * draw_multiplier));

    if(flags & DrawMatchesFlags::DRAW_RICH_KEYPOINTS) {
        int radius = cvRound(p.size/2 * draw_multiplier); // KeyPoint::size is a diameter

        // draw the circles around keypoints with the keypoints size
        circle(img, center, radius, color, 1, CV_AA, draw_shift_bits);

        // draw orientation of the keypoint, if it is applicable
        if(p.angle != -1) {
            float srcAngleRad = p.angle*(float)CV_PI/180.f;
            Point orient(cvRound(cos(srcAngleRad)*radius),
                         cvRound(sin(srcAngleRad)*radius)
                        );
            line(img, center, center+orient, color, 1, CV_AA, draw_shift_bits);
        }
#if 0
        else {
            // draw center with R=1
            int radius = 1 * draw_multiplier;
            circle(img, center, radius, color, 1, CV_AA, draw_shift_bits);
        }
#endif
    } else {
        // draw center with R=3
        int radius = 3 * draw_multiplier;
        circle(img, center, radius, color, 1, CV_AA, draw_shift_bits);
    }
}

static inline void drawConvexHull(const std::vector<Point>& hull, Mat& img, const Scalar& color)
{
    const Point* last = &hull[hull.size()-1];
    for(unsigned j = 0; j < hull.size(); ++j) {
        line(img, *last, hull[j], color, 1, CV_AA);
        last = &hull[j];
    }
}
}

cv::Mat ImageCombinerClusterMatch::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    Frame::Ptr a = FrameIO::convert(img1, mask1);
    Frame::Ptr b = FrameIO::convert(img2, mask2);

    Frame::ExtractorFunction extractor = boost::bind(&Extractor::extract, tools->getExtractor(), _1, _2, _3, _4);
    a->extractFeatures(extractor);
    b->extractFeatures(extractor);

    std::vector<std::vector<cv::DMatch> > matches;
    tools->getMatcher()->match(a.get(), b.get(), matches);

    WARN("got " << matches.size() << " matches");


    ClusteringOptions* opt = get<Option, ClusteringOptions>();
    assert(opt);
    WARN("k=" << opt->k);
    Hough h(opt->k, config.octaves, a, b);

    std::vector<KMeansAlgorithm::ClusterT> clusters;
    h.filter(matches, clusters);

    for(typename std::vector<KMeansAlgorithm::ClusterT>::iterator cluster = clusters.begin(); cluster != clusters.end(); ++cluster) {
        for(typename std::vector<KMeansAlgorithm::VectorT*>::iterator v = cluster->members.begin(); v != cluster->members.end(); ++v) {
            assert((*v)->data != NULL);
            assert((*v)->data != (cv::DMatch*) 0xDEADBEEF);
            cv::DMatch m = *(*v)->data;
            assert(m.queryIdx > -1);
        }
    }

    cv::Mat out;

//    cv::Scalar matchColor = cv::Scalar::all(255);
//    cv::Scalar singlePointColor = cv::Scalar(192, 64, 64);
//    std::vector<std::vector<char> > mask;
//    int flag = cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;

    try {
        //  cv::drawMatches(img1, a->keypoints, img2, b->keypoints, matches, out, matchColor, singlePointColor, mask, flag);
        int debug_w = 300;
        out = cv::Mat(std::max(img1.rows, img2.rows), img1.cols + img2.cols + debug_w, img1.type(), cv::Scalar::all(0));

        int offset_x = img1.cols;

        cv::Mat target_img1(out, cv::Rect(debug_w, 0, img1.cols, img1.rows));
        cv::Mat target_img2(out, cv::Rect(offset_x + debug_w, 0, img2.cols, img2.rows));
        cv::Mat combined_target(out, cv::Rect(debug_w, 0, img2.cols + img1.cols, img2.rows));

        img1.copyTo(target_img1);
        img2.copyTo(target_img2);

//        track clusters -> same cluster -> same color!

        for(unsigned i = 0; i < clusters.size(); ++i) {
            KMeansAlgorithm::ClusterT& cluster = clusters[i];
            if(cluster.members.size() < 4) {
                continue;
            }

            std::cout << "cluster " << i << ": " << cluster.members.size() << std::endl;

            KMeansAlgorithm::VectorT center = cluster.centroid;

            cv::Scalar c = color(i, clusters.size());
            cv::circle(h.debug, cv::Point(center[2], center[3]), 10, c, 1, CV_AA);

            int max_dist = 5;

            std::vector<cv::Point> points_a, points_b;
            for(unsigned j = 0; j < cluster.members.size(); ++j) {
                KMeansAlgorithm::VectorT* member = cluster.members[j];

                cv::DMatch match = *member->data;

                if(KMeansAlgorithm::Distance::distance(*member, center) > max_dist) {
                    cv::drawKeypoint(target_img1, a->keypoints[match.queryIdx], cv::Scalar::all(0), 0);
                    cv::drawKeypoint(target_img2, b->keypoints[match.trainIdx], cv::Scalar::all(0), 0);
                    continue;
                }

                cv::drawKeypoint(target_img1, a->keypoints[match.queryIdx], c, cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                cv::drawKeypoint(target_img2, b->keypoints[match.trainIdx], c, cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                points_a.push_back(a->keypoints[match.queryIdx].pt);
                points_b.push_back(b->keypoints[match.trainIdx].pt);
            }

            if(points_a.size() >= 4) {

                std::vector<cv::Point> hull_a, hull_b;
                cv::convexHull(points_a, hull_a);
                cv::convexHull(points_b, hull_b);

                cv::drawConvexHull(hull_a, target_img1, c);
                cv::drawConvexHull(hull_b, target_img2, c);

                for(unsigned j = 0; j < cluster.members.size(); ++j) {
                    KMeansAlgorithm::VectorT* member = cluster.members[j];
                    if(KMeansAlgorithm::Distance::distance(*member, center) > max_dist) {
                        continue;
                    }

                    cv::DMatch match = *member->data;

                    cv::Point2f& target = b->keypoints[match.trainIdx].pt;
//                    cv::line(combined_target, a->keypoints[match.queryIdx].pt, cv::Point(target.x + offset_x, target.y), c, 1, CV_AA);
                }
            }
        }

        cv::Size size(debug_w, h.debug.rows * (double) debug_w / h.debug.cols);
        cv::resize(h.debug, cv::Mat(out, cv::Rect(0, 0, size.width, size.height)), size);
//        h.debug.copyTo(cv::Mat(out, cv::Rect(0, 0, debug_w, h.debug.rows * h.debug.cols / (double) debug_w)));

    } catch(cv::Exception& e) {
        WARN("cv exception: " << e.what());
    }

    return out;
}

ImageCombiner::TypePtr ImageCombinerClusterMatch::createInstance(CONSTRUCTOR_MODE mode)
{
    return ImageCombiner::TypePtr(new ImageCombinerClusterMatch("Cluster Match KMeans"));
}
