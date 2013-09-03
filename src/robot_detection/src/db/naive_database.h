#ifndef NAIVE_DATABASE_H
#define NAIVE_DATABASE_H

/// COMPONENT
#include "database.h"

/// SYSTEM
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>

/**
 * @brief The NaiveDatabase class implements a simple database
 */
class NaiveDatabase : public Database
{
public:
    /**
     * @brief NaiveDatabase
     */
    NaiveDatabase();

    /**
     * @brief add Add pose
     * @param pose
     */
    void add(MatchablePose* pose);

    /**
     * @brief traversePoses traverses the complete database and calls the callbacks
     * @param composite_callback is called for every node in the tree
     * @param leaf_callback is called for every leaf / pose
     */
    virtual void traversePoses(boost::function<void(int,const std::string&)> composite_callback, boost::function<void(int,MatchablePose*)> leaf_callback);

    /**
     * @brief deletePoseByIndex
     * @param index
     */
    void deletePoseByIndex(const int index);

    /**
     * @brief clear resets everything
     */
    void clear();

    /**
     * @brief train finishes the training
     */
    void finishTraining();

protected:
    double findBestMatch(Matchable* current_frame, MatchablePose *&out, int* no_of_features = NULL) const;

private:
    std::vector<MatchablePose*> poses;


private:
    BOOST_SERIALIZATION_SPLIT_MEMBER()

    friend class boost::serialization::access;

    template<class Archive>
    void save(Archive& ar, const unsigned int /*version*/) const {
        ar& boost::serialization::base_object<Database>(*this);

        ar << poses;

        for(std::vector<MatchablePose*>::const_iterator it = poses.begin(); it != poses.end(); ++it) {
            (*it)->saved = true;
        }
    }

    template<class Archive>
    void load(Archive& ar, const unsigned int /*version*/) {
        poses.clear();
        ar >> poses;

        for(std::vector<MatchablePose*>::const_iterator it = poses.begin(); it != poses.end(); ++it) {
            (*it)->saved = true;
        }
    }
};

#endif // NAIVE_DATABASE_H
