/// HEADER
#include "image_combiner_cluster_match_em.h"

/// COMPONENT
#include "option_clustering.h"

/// PROJECT
#include <data/frame_io.h>
#include <utils/opencv_utils.hpp>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(robot_detection::ImageCombinerClusterMatchEM, csapex::BoxedObject)

using namespace lib_clustering;
using namespace robot_detection;

class HoughEM
{

public:
    HoughEM(int cluster_count, int octaves, Frame::Ptr a, Frame::Ptr b)
        : k(cluster_count), algo(k), octaves(octaves), a(a), b(b) {
        padding = 1;
        scaling = 4;

        dim_theta = 360 / 30;
        dim_sigma = 2 * octaves;
        dim_tx = 2 * std::max(a->getWidth(), b->getWidth()) / scaling;
        dim_ty = 2 * std::max(a->getHeight(), b->getHeight()) / scaling;

        dim1 = dim_theta;
        dim2 = dim1 * dim_sigma;
        dim3 = dim2 * dim_tx;

        hough = new unsigned[(dim_theta+2*padding) * (dim_sigma+2*padding) * (dim_tx+2*padding) * (dim_ty+2*padding)];
    }

    ~HoughEM() {
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

                PointT pt(std::vector<int>(), std::pair<unsigned, cv::DMatch*>(1, &match));
                pt.first.push_back(itheta);
                pt.first.push_back(isigma);
                pt.first.push_back(itx);
                pt.first.push_back(ity);
                candidates.push_back(pt);

                debug.at<cv::Vec3b>(padding + ity, padding + itx) -= cv::Vec3b::all(64);

//                for(int s = -padding; s <= padding; ++s){
//                    for(int t = -padding; t <= padding; ++t){
//                        for(int u = -padding; u <= padding; ++u){
//                            for(int v = -padding; v <= padding; ++v){
//                                vote(itheta+s, isigma+t, itx+u, ity+v);
//                            }
//                        }
//                    }
//                }
            }
        }

        if((int) candidates.size() < k) {
            WARN("cannot cluster, not enough matches");
            return;
        }

        candidates_cv = cv::Mat(candidates.size(), 4, CV_32FC1);
        int row = 0;
        for(KMeansAlgorithm::InputDataTypeT::iterator it = candidates.begin(); it != candidates.end(); ++it) {
            PointT& pt = *it;

            for(int i = 0; i < 4; ++i) {
                candidates_cv.at<float>(row, i) = pt.first[i];
            }

            ++row;
        }
//        cv::EM em(k, cv::EM::COV_MAT_GENERIC);
        cv::EM em(k, cv::EM::COV_MAT_DIAGONAL);
        cv::Mat labels;
        em.train(candidates_cv, cv::noArray(), labels);

        clusters.clear();
        cv::Mat centers = em.get<cv::Mat>("means");
        std::vector<cv::Mat> covs = em.get<std::vector<cv::Mat> >("covs");

        for(int j = 0; j < labels.rows; ++j) {
            cv::Mat c = candidates_cv.row(j);
            assert(c.type() == CV_32FC1);
            KMeansAlgorithm::VectorT v(0);
            for(int col = 0; col < c.cols; ++col) {
                v[col] = c.at<float>(0, col);
            }
            v.data = &matches[j][0];
            candidates_vector.push_back(v);
        }


        for(int center_row = 0; center_row < centers.rows; ++center_row) {
            cv::Mat center = centers.row(center_row);
            assert(center.type() == CV_64FC1);

            KMeansAlgorithm::VectorT v(0);
            for(int col = 0; col < center.cols; ++col) {
                v[col] = center.at<double>(0, col);
            }
            KMeansAlgorithm::ClusterT cluster(v);
            for(int label_row = 0; label_row < labels.rows; ++label_row) {
                if(labels.at<int>(label_row) == center_row) {
                    cluster.members.push_back(&candidates_vector[label_row]);
                }
            }
            clusters.push_back(cluster);
//            cv::circle(debug, cv::Point(v[2], v[3]), 20, cv::Scalar::all(64), 1, CV_AA);

            cv::Mat eigenvalues;
            cv::Mat eigenvectors;

            cv::Mat cov = covs[center_row].rowRange(2,4).colRange(2,4);

            cv::eigen(cov, eigenvalues, eigenvectors);

            std::cout << cov << std::endl;
            std::cout << eigenvalues << std::endl;
            std::cout << eigenvectors << std::endl;

            assert(eigenvalues.type() == CV_64FC1);

            double lambda1 = eigenvalues.at<double>(0,0);
            double lambda2 = eigenvalues.at<double>(1,0);

            assert(eigenvectors.type() == CV_64FC1);

            double eigenvec1_x   = eigenvectors.at<double>(0,0) * lambda1;
            double eigenvec1_y   = eigenvectors.at<double>(0,1) * lambda1;

            cv::Size size(std::sqrt(lambda1)*3, std::sqrt(lambda2)*3);
            double angle = atan2(eigenvec1_y, eigenvec1_x) * (180.0/M_PI);

            std::cout << angle << std::endl;
            cv::ellipse(debug, cv::RotatedRect(cv::Point(v[2], v[3]), size, angle), cv::Scalar::all(64), 1, CV_AA);
        }

        std::cout << "labels=" << labels.rows << ", centers=" << centers.rows << ", clusters=" << clusters.size() << std::endl;
    }

    inline void vote(int itheta, int isigma, int itx, int ity) {
        assert(0 <= ity + padding);
        assert(ity < debug.rows + padding);
        assert(0 <= itx + padding);
        assert(itx < debug.cols + padding);
        //hough[(itheta + padding) + (isigma + padding) * dim1 + (itx + padding) * dim2 + (ity + padding) * dim3]++;
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

    cv::Mat candidates_cv;
    std::vector<KMeansAlgorithm::VectorT> candidates_vector;
    KMeansAlgorithm::InputDataTypeT candidates;

    unsigned* hough;

public:
    cv::Mat debug;
};

ClusteringOptions ImageCombinerClusterMatchEM::options;

void ImageCombinerClusterMatchEM::update_gui(QFrame* additional_holder)
{
    QBoxLayout* layout = new QVBoxLayout;
    additional_holder->setLayout(layout);

    insert(layout);
}

void ImageCombinerClusterMatchEM::insert(QBoxLayout *layout)
{
    options.insert(layout);
}


cv::Mat ImageCombinerClusterMatchEM::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
    Frame::Ptr a = FrameIO::convert(img1, mask1);
    Frame::Ptr b = FrameIO::convert(img2, mask2);

    Frame::ExtractorFunction extractor = boost::bind(&Extractor::extract, tools->getExtractor(), _1, _2, _3, _4);
    a->extractFeatures(extractor);
    b->extractFeatures(extractor);

    std::vector<std::vector<cv::DMatch> > matches;
    tools->getMatcher()->match(a.get(), b.get(), matches);

    WARN("got " << matches.size() << " matches");

    HoughEM h(options.k, config.octaves, a, b);

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
        out = cv::Mat(std::max(img1.rows, img2.rows), img1.cols + img2.cols + h.debug.cols, img1.type(), cv::Scalar::all(0));

        int offset_x = img1.cols;

        cv::Mat target_img1(out, cv::Rect(h.debug.cols, 0, img1.cols, img1.rows));
        cv::Mat target_img2(out, cv::Rect(offset_x + h.debug.cols, 0, img2.cols, img2.rows));
        cv::Mat combined_target(out, cv::Rect(h.debug.cols, 0, img2.cols + img1.cols, img2.rows));

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

            cv::Scalar c = cv::rangeColor(i, clusters.size());
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

//                    cv::DMatch match = *member->data;

//                    cv::Point2f& target = b->keypoints[match.trainIdx].pt;
//                    cv::line(combined_target, a->keypoints[match.queryIdx].pt, cv::Point(target.x + offset_x, target.y), c, 1, CV_AA);
                }
            }
        }

        h.debug.copyTo(cv::Mat(out, cv::Rect(0, 0, h.debug.cols, h.debug.rows)));

    } catch(cv::Exception& e) {
        WARN("cv exception: " << e.what());
    }

    return out;
}
