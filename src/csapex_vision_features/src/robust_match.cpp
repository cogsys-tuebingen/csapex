/*
 * image_combiner_robust_match.cpp
 *
 *  Created on: Feb 7, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// HEADER
#include "robust_match.h"

/// COMPONENT
#include <csapex_vision_features/keypoint_message.h>
#include <csapex_vision_features/descriptor_message.h>

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_in.h>
#include <csapex_vision/cv_mat_message.h>

/// SYSTEM
#include <csapex/utility/register_apex_plugin.h>
#include <opencv2/opencv.hpp>

CSAPEX_REGISTER_CLASS(csapex::RobustMatch, csapex::BoxedObject)

using namespace csapex;
using namespace connection_types;

//RobustMatcher class taken from OpenCV2 Computer Vision Application Programming Cookbook Ch 9
class RobustMatcher
{
private:
    // pointer to the matcher object
    cv::Ptr<cv::DescriptorMatcher > matcher;
    float ratio; // max ratio between 1st and 2nd NN
    bool refineF; // if true will refine the F matrix
    double confidence; // confidence level (probability)
    double distance; // min distance to epipolar
public:
    RobustMatcher(cv::Ptr<cv::DescriptorMatcher > m) : ratio(0.75f), refineF(true),
        confidence(0.99), distance(3.0) {
        matcher= m;
    }

    // Set the matcher
    void setDescriptorMatcher(
        cv::Ptr<cv::DescriptorMatcher>& match) {
        matcher= match;
    }
    // Set confidence level
    void setConfidenceLevel(
        double conf) {
        confidence= conf;
    }
    //Set MinDistanceToEpipolar
    void setMinDistanceToEpipolar(
        double dist) {
        distance= dist;
    }
    //Set ratio
    void setRatio(
        float rat) {
        ratio= rat;
    }

    // Clear matches for which NN ratio is > than threshold
    // return the number of removed points
    // (corresponding entries being cleared,
    // i.e. size will be 0)
    int ratioTest(std::vector<std::vector<cv::DMatch> >
                  &matches) {
        int removed=0;
        // for all matches
        for(std::vector<std::vector<cv::DMatch> >::iterator
                matchIterator= matches.begin();
                matchIterator!= matches.end(); ++matchIterator) {
            // if 2 NN has been identified
            if(matchIterator->size() > 1) {
                // check distance ratio
                if((*matchIterator)[0].distance/
                        (*matchIterator)[1].distance > ratio) {
                    matchIterator->clear(); // remove match
                    removed++;
                }
            } else { // does not have 2 neighbours
                matchIterator->clear(); // remove match
                removed++;
            }
        }
        return removed;
    }

    // Insert symmetrical matches in symMatches vector
    void symmetryTest(
        const std::vector<std::vector<cv::DMatch> >& matches1,
        const std::vector<std::vector<cv::DMatch> >& matches2,
        std::vector<cv::DMatch>& symMatches) {
        // for all matches image 1 -> image 2
        for(std::vector<std::vector<cv::DMatch> >::
                const_iterator matchIterator1= matches1.begin();
                matchIterator1!= matches1.end(); ++matchIterator1) {
            // ignore deleted matches
            if(matchIterator1->size() < 2)
                continue;
            // for all matches image 2 -> image 1
            for(std::vector<std::vector<cv::DMatch> >::
                    const_iterator matchIterator2= matches2.begin();
                    matchIterator2!= matches2.end();
                    ++matchIterator2) {
                // ignore deleted matches
                if(matchIterator2->size() < 2)
                    continue;
                // Match symmetry test
                if((*matchIterator1)[0].queryIdx ==
                        (*matchIterator2)[0].trainIdx &&
                        (*matchIterator2)[0].queryIdx ==
                        (*matchIterator1)[0].trainIdx) {
                    // add symmetrical match
                    symMatches.push_back(
                        cv::DMatch((*matchIterator1)[0].queryIdx,
                                   (*matchIterator1)[0].trainIdx,
                                   (*matchIterator1)[0].distance));
                    break; // next match in image 1 -> image 2
                }
            }
        }
    }

    // Identify good matches using RANSAC
    // Return fundemental matrix
    cv::Mat ransacTest(
        const std::vector<cv::DMatch>& matches,
        const std::vector<cv::KeyPoint>& keypoints1,
        const std::vector<cv::KeyPoint>& keypoints2,
        std::vector<cv::DMatch>& outMatches) {
        // Convert keypoints into Point2f
        std::vector<cv::Point2f> points1, points2;
        cv::Mat fundemental;
        for(std::vector<cv::DMatch>::
                const_iterator it= matches.begin();
                it!= matches.end(); ++it) {
            // Get the position of left keypoints
            float x= keypoints1[it->queryIdx].pt.x;
            float y= keypoints1[it->queryIdx].pt.y;
            points1.push_back(cv::Point2f(x,y));
            // Get the position of right keypoints
            x= keypoints2[it->trainIdx].pt.x;
            y= keypoints2[it->trainIdx].pt.y;
            points2.push_back(cv::Point2f(x,y));
        }
        // Compute F matrix using RANSAC
        std::vector<uchar> inliers(points1.size(),0);
        if(points1.size()>0&&points2.size()>0) {
            cv::Mat fundemental= cv::findFundamentalMat(
                                     cv::Mat(points1),cv::Mat(points2), // matching points
                                     inliers,       // match status (inlier or outlier)
                                     CV_FM_RANSAC, // RANSAC method
                                     distance,      // distance to epipolar line
                                     confidence); // confidence probability
            // extract the surviving (inliers) matches
            std::vector<uchar>::const_iterator
            itIn= inliers.begin();
            std::vector<cv::DMatch>::const_iterator
            itM= matches.begin();
            // for all matches
            for(; itIn!= inliers.end(); ++itIn, ++itM) {
                if(*itIn) {  // it is a valid match
                    outMatches.push_back(*itM);
                }
            }
            if(refineF) {
                // The F matrix will be recomputed with
                // all accepted matches
                // Convert keypoints into Point2f
                // for final F computation
                points1.clear();
                points2.clear();
                for(std::vector<cv::DMatch>::
                        const_iterator it= outMatches.begin();
                        it!= outMatches.end(); ++it) {
                    // Get the position of left keypoints
                    float x= keypoints1[it->queryIdx].pt.x;
                    float y= keypoints1[it->queryIdx].pt.y;
                    points1.push_back(cv::Point2f(x,y));
                    // Get the position of right keypoints
                    x= keypoints2[it->trainIdx].pt.x;
                    y= keypoints2[it->trainIdx].pt.y;
                    points2.push_back(cv::Point2f(x,y));
                }
                // Compute 8-point F from all accepted matches
                if(points1.size()>0&&points2.size()>0) {
                    fundemental= cv::findFundamentalMat(
                                     cv::Mat(points1),cv::Mat(points2), // matches
                                     CV_FM_8POINT); // 8-point method
                }
            }
        }
        return fundemental;
    }

    // Match feature points using symmetry test and RANSAC
    // returns fundemental matrix
    cv::Mat match(const cv::Mat& image1,
                  const cv::Mat& image2,
                  std::vector<cv::DMatch>& matches,
                  std::vector<cv::KeyPoint>& keypoints1,
                  std::vector<cv::KeyPoint>& keypoints2,
                  cv::Mat& descriptors1,
                  cv::Mat& descriptors2) {
        // 2. Match the two image descriptors
        // Construction of the matcher
        //cv::BruteForceMatcher<cv::L2<float>> matcher;
        // from image 1 to image 2
        // based on k nearest neighbours (with k=2)
//        std::cerr << "d1: \n";
//        for(size_t r = 0; r < descriptors1.rows; ++r) {
//            for(size_t c = 0; c < descriptors1.cols; ++c) {
//                std::cerr << descriptors1.at<float>(r,c) <<"\t";
//            }
//            std::cerr << "\n";
//        }

//        std::cerr << std::endl;


//        std::cerr << "d2: \n";
//        for(size_t r = 0; r < descriptors2.rows; ++r) {
//            for(size_t c = 0; c < descriptors2.cols; ++c) {
//                std::cerr << descriptors2.at<float>(r,c) <<"\t";
//            }
//            std::cerr << "\n";
//        }

//        std::cerr << std::endl;

        std::vector<std::vector<cv::DMatch> > matches1;
        matcher->knnMatch(descriptors1,descriptors2,
                          matches1, // vector of matches (up to 2 per entry)
                          2);        // return 2 nearest neighbours

        // from image 2 to image 1
        // based on k nearest neighbours (with k=2)
        std::vector<std::vector<cv::DMatch> > matches2;
        matcher->knnMatch(descriptors2,descriptors1,
                          matches2, // vector of matches (up to 2 per entry)
                          2);        // return 2 nearest neighbours
        // 3. Remove matches for which NN ratio is
        // > than threshold
        // clean image 1 -> image 2 matches
        // clean image 2 -> image 1 matches
        ratioTest(matches1);
        ratioTest(matches2);

        // 4. Remove non-symmetrical matches
        std::vector<cv::DMatch> symMatches;
        symmetryTest(matches1,matches2,symMatches);

        // 5. Validate matches using RANSAC
        cv::Mat fundemental= ransacTest(symMatches,
                                        keypoints1, keypoints2, matches);
        // return the found fundemental matrix
        return fundemental;
    }
};

RobustMatch::RobustMatch()
    : in_img_1(NULL)
{
    addTag(Tag::get("Features"));
}


void RobustMatch::allConnectorsArrived()
{
    CvMatMessage::Ptr img1 = in_img_1->getMessage<CvMatMessage>();
    CvMatMessage::Ptr img2 = in_img_2->getMessage<CvMatMessage>();

    KeypointMessage::Ptr key1 = in_key_1->getMessage<KeypointMessage>();
    KeypointMessage::Ptr key2 = in_key_2->getMessage<KeypointMessage>();

    DescriptorMessage::Ptr des1 = in_des_1->getMessage<DescriptorMessage>();
    DescriptorMessage::Ptr des2 = in_des_2->getMessage<DescriptorMessage>();

    std::vector<cv::DMatch> matches;

    cv::Ptr<cv::DescriptorMatcher > matcher;
    if(des1->value.type() == CV_8U) {
        matcher = new cv::BFMatcher(cv::NORM_HAMMING);
    } else {
        matcher = new cv::BFMatcher(cv::NORM_L2);
    }
    RobustMatcher m(matcher);
    m.match(img1->value, img2->value, matches, key1->value, key2->value, des1->value, des2->value);

    CvMatMessage::Ptr out(new CvMatMessage);
    cv::drawMatches(img1->value, key1->value, img2->value, key2->value, matches, out->value, cv::Scalar(0,0,255), cv::Scalar::all(0), cv::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    out_img->publish(out);
}


void RobustMatch::fill(QBoxLayout* layout)
{
    if(in_img_1 == NULL) {
        box_->setSynchronizedInputs(true);

        in_img_1 = box_->addInput<CvMatMessage>("Image 1");
        in_key_1 = box_->addInput<KeypointMessage>("Keypoints 1");
        in_des_1 = box_->addInput<DescriptorMessage>("Descriptor 1");

        in_img_2 = box_->addInput<CvMatMessage>("Image 2");
        in_key_2 = box_->addInput<KeypointMessage>("Keypoints 2");
        in_des_2 = box_->addInput<DescriptorMessage>("Descriptor 2");

        out_img = box_->addOutput<CvMatMessage>("Debug View");
    }
}
