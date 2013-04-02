#ifndef BAG_DATABASE_H
#define BAG_DATABASE_H

/// COMPONENT
#include "database.h"
#include "bag.h"

class BagDatabase : public Database
{
public:
    /**
     * @brief BinDatabase
     */
    BagDatabase();

    /**
     * @brief ~BagDatabase
     */
    ~BagDatabase();

    /**
     * @brief getPoseByAngle Find the pose closest to a given angle
     * @param yaw target angle
     * @param index Output: index of the pose
     * @return the best pose
     */
    virtual MatchablePose* getPoseByAngle(const double yaw, int* index = NULL) const;

    /**
     * @brief findBestMatch Find the best pose for the given frame
     * @param current_frame target frame
     * @param out Output: Pose found or Pose::NULL_POSE
     * @param no_of_features Output: Number of features found
     * @return score of the best pose
     */
    virtual double findBestMatch(Matchable* current_frame, MatchablePose *&out, int* no_of_features = NULL) const;

    /**
     * @brief add add a pose to the database
     * @param pose
     */
    virtual void add(MatchablePose* pose);

    /**
     * @brief clear clears the database
     */
    virtual void clear();

    /**
     * @brief train finishes the training
     */
    void finishTraining();

protected:
    /**
     * @brief config_changed Callback when the config has changed
     */
    void configChanged();

private:
    Bag* bag;


private:

    BOOST_SERIALIZATION_SPLIT_MEMBER()

    friend class boost::serialization::access;

    template<class Archive>
    void save(Archive& ar, const unsigned int version) const {
        ar << *bag;
    }

    template<class Archive>
    void load(Archive& ar, const unsigned int version) {
        // @TODO
    }
};

#endif // BAG_DATABASE_H
