#ifndef BIN_DATABASE_H
#define BIN_DATABASE_H

/// COMPONENT
#include "bin.h"
#include "database.h"

/// SYSTEM
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>

/**
 * @brief The BinDatabase class implements a histogram database
 */
class BinDatabase : public Database
{
    friend class BinDatabaseStrategy;
    friend class Bin;

public:
    /**
     * @brief BinDatabase
     */
    BinDatabase();

    /**
     * @brief traversePoses traverses the complete database and calls the callbacks
     * @param composite_callback is called for every node in the tree
     * @param leaf_callback is called for every leaf / pose
     */
    virtual void traversePoses(boost::function<void(int,const std::string&)> composite_callback, boost::function<void(int,MatchablePose*)> leaf_callback);

    /**
     * @brief add add a pose to the database
     * @param pose
     */
    void add(MatchablePose* pose);

    /**
     * @brief addValidationExample add a validation pose to the database, used for training
     * @param m
     */
    void addValidationExample(Matchable* m);

    /**
     * @brief finishTraining finishes the training
     */
    void finishTraining();

    /**
     * @brief clear resets everything
     */
    void clear();

protected:
    double findBestMatch(Matchable* current_frame, MatchablePose *&out, int* no_of_features = NULL) const;

private:
    void makeBins(int max_dist);
    int angle2index(double angle) const;

private:
    int bin_max_dist_;
    std::vector<Bin> bins;


private:
    BOOST_SERIALIZATION_SPLIT_MEMBER()

    friend class boost::serialization::access;

    template<class Archive>
    void save(Archive& ar, const unsigned int /*version*/) const {
        ar& boost::serialization::base_object<Database>(*this);

        ar << bins;

        for(std::vector<Bin>::const_iterator it = bins.begin(); it != bins.end(); ++it) {
            (*it).setSaved(true);
        }
    }

    template<class Archive>
    void load(Archive& ar, const unsigned int /*version*/) {
        ar& boost::serialization::base_object<Database>(*this);

        bins.clear();
        ar >> bins;

        config["bin_count"] = static_cast<int>(bins.size());
        config.replaceInstance();

        for(std::vector<Bin>::const_iterator it = bins.begin(); it != bins.end(); ++it) {
            (*it).setSaved(true);
        }
    }
};

#endif // BIN_DATABASE_H
