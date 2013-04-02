#ifndef MATCHABLE_POSE_H
#define MATCHABLE_POSE_H

/// COMPONENT
#include "matchable.h"
#include "frame.h"

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/split_member.hpp>
#include <opencv2/opencv.hpp>

/**
 * @brief The Pose class represents a robot pose
 */
class MatchablePose : public Matchable
{
public:
    static std::string IMAGE_PATH;
    static MatchablePose* NULL_POSE;

    struct LessThan {
        inline bool operator()(const MatchablePose* a, const MatchablePose* b) {
            return (a->distance < b->distance);
        }
        inline bool operator()(const MatchablePose& a, const MatchablePose& b) {
            return (a.distance < b.distance);
        }
    };
    struct MoreThan {
        inline bool operator()(const MatchablePose* a, const MatchablePose* b) {
            return (a->distance > b->distance);
        }
        inline bool operator()(const MatchablePose& a, const MatchablePose& b) {
            return (a.distance > b.distance);
        }
    };

public:
    /**
     * @brief Pose empty constructor
     */
    MatchablePose();

    /**
     * @brief Pose copy constructor
     * @param pose
     */
    MatchablePose(const MatchablePose& pose);

    /**
     * @brief Pose Convert a Frame into a Pose
     * @param frame
     */
    MatchablePose(const Frame::Ptr frame);

    /**
     * @brief get_dimensions
     * @return the width and height of this matchable
     */
    virtual cv::Rect getDimensions() const;

public:
    cv::Mat image;
    cv::Mat mask;

    bool saved;

private:
    static long IMAGE_ID;

private:
    inline std::string pose_to_image_name(int id) const {
        double deg = std::floor(orientation.toDegrees());
        std::stringstream path;
        path << MatchablePose::IMAGE_PATH + "/pose_" << deg << "deg_" << id << ".png";

        return path.str();
    }


private:
    BOOST_SERIALIZATION_SPLIT_MEMBER()

    friend class boost::serialization::access;

    template<class Archive>
    void save(Archive& ar, const unsigned int version) const {
        ar& descriptors;
        ar& keypoints;

        double yaw = orientation.toRadians();
        ar& yaw;
        ar& distance;
        ar& IMAGE_ID;

        std::string name = pose_to_image_name(IMAGE_ID);

        IMAGE_ID++;

        cv::Mat feat;
        cv::drawKeypoints(image, keypoints, feat, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        cv::imwrite(name, image);
        cv::imwrite(name + "_features.png", feat);
    }

    template<class Archive>
    void load(Archive& ar, const unsigned int version) {
        ar& descriptors;
        ar& keypoints;

        double yaw;
        ar& yaw;
        orientation = Angle(yaw);

        ar& distance;

        long image_id;
        ar& image_id;

        std::string path = pose_to_image_name(image_id);
        image = cv::imread(path);
    }
};

#endif // MATCHABLE_POSE_H
