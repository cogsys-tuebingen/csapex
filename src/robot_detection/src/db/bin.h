#ifndef BIN_H
#define BIN_H

/// PROJECT
#include <config/reconfigurable.h>

/// SYSTEM
#include <assert.h>
#include <boost/serialization/access.hpp>
#include <vector>

/// FORWARD DECLARATIONS
class BinDatabase;
class Database;
class MatchablePose;
class Matchable;
class Matcher;
class MatchScorer;

/// SERIALIZATION

namespace boost
{
namespace archive
{

class polymorphic_iarchive;
class polymorphic_oarchive;

}
}

/**
 * @brief The Bin class represents a bin in the histogram of a BinDatabase
 */
class Bin : public Reconfigurable
{
    friend class BinDatabase;

public:
    /**
     * @brief Bin
     * @param angle_low lower bound (inclusive)
     * @param angle_high upper bound (exclusive)
     * @param max_pose_count maximum number of entries
     */
    Bin(double angle_low = -M_PI, double angle_high = +M_PI, int max_pose_count = 20);

    /**
     * @brief ~Bin
     */
    virtual ~Bin();

    /**
     * @brief add insert a pose into this bin
     * @param pose
     */
    void add(MatchablePose* pose);

    /**
     * @brief addValidationExample add a validation pose, used for trainng
     * @param m
     */
    void addValidationExample(Matchable* m);

    /**
     * @brief setSaved Set state to saved
     * @param saved
     */
    void setSaved(bool saved) const;

    /**
     * @brief size
     * @return the number of poses in this bin
     */
    int size() const;

    /**
     * @brief filter filter this bin
     */
    void filter();

    /**
     * @brief merge merge all poses of this bin
     */
    void merge();

    /**
     * @brief findBestMatch find the best pose for the given frame
     * @param current_frame the frame to look for
     * @param out Output: The pose, if one was found, else Pose::NULL_POSE
     * @param no_of_features Output: The number of features found
     * @return the score of the best pose or INFINITY of none was found
     */
    double findBestMatch(Matchable* current_frame, MatchablePose *&out, int* no_of_features = 0) const;

    /**
     * @brief dump saves all images to the given path
     * @param path
     */
    void dump(const std::string& path) const;

private:
    void sortPosesByDistance();

private:
    std::vector<MatchablePose*> poses;
    std::vector<const Matchable*> validation;

    double angle_low;
    double angle_high;
    int max_pose_count;

    mutable cv::Mat best_debug;

private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& angle_low;
        ar& angle_high;
        ar& poses;
    }
};

#endif // BIN_H
