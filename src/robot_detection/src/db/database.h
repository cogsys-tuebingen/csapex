#ifndef DATABASE_H
#define DATABASE_H

/// COMPONENT
#include <config/reconfigurable.h>

/// PROJECT
#include <data/matchable_pose.h>

/// SYSTEM
#include <boost/signals2.hpp>
#include <opencv2/opencv.hpp>

/// FORWARD DECLARATION
class Extractor;
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
 * @brief The Database class is an abstract base class for different database models
 */
class Database : public Reconfigurable
{

public:
    /**
     * @brief Database
     */
    Database();

    /**
     * @brief ~Database
     */
    virtual ~Database();

    /**
     * @brief getBestMatch find the best pose for the given frame
     * @param current_frame target frame
     * @param score best score found
     * @param no_of_features number of features found
     * @return best pose found or Pose::NULL_POSE
     */
    virtual MatchablePose* getBestMatch(Matchable* current_frame, double* score = NULL, int* no_of_features = NULL) const;

    /**
     * @brief traversePoses traverses the complete database and calls the callbacks
     * @param composite_callback is called for every node in the tree
     * @param leaf_callback is called for every leaf / pose
     */
    virtual void traversePoses(boost::function<void(int,const std::string&)> composite_callback, boost::function<void(int,MatchablePose*)> leaf_callback);

public: // abstract
    /**
     * @brief findBestMatch Find the best pose for the given frame
     * @param current_frame target frame
     * @param out Output: Pose found or Pose::NULL_POSE
     * @param no_of_features Output: Number of features found
     * @return score of the best pose
     */
    virtual double findBestMatch(Matchable* current_frame, MatchablePose *&out, int* no_of_features = NULL) const = 0;

    /**
     * @brief add add a pose to the database
     * @param pose
     */
    virtual void add(MatchablePose* pose) = 0;

    /**
     * @brief clear clears the database
     */
    virtual void clear() = 0;

    /**
     * @brief train finishes the training
     */
    virtual void finishTraining() = 0;

public:
    /**
     * @brief changed Signals that the database has changed
     */
    boost::signals2::signal<void ()> changed;

    /**
     * @brief replaced Signals that the database has been replaced
     */
    boost::signals2::signal<void ()> replaced;


protected:
    double threshold_score;

public:
    mutable cv::Mat debug;

private:
    friend class boost::serialization::access;

    template<typename Archive>
    void serialize(Archive& ar, const unsigned int file_version) {
    }
};

#endif // DATABASE_H
