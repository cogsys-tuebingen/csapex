#ifndef BOW_DATABASE_H
#define BOW_DATABASE_H

/// COMPONENT
#include "database.h"

/// SYSTEM
#include <opencv2/opencv.hpp>

class BowDatabase: public Database
{
public:
    /**
     * @brief BinDatabase
     */
    BowDatabase();

    /**
     * @brief ~BagDatabase
     */
    ~BowDatabase();

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
     * @brief finishTraining finishes the training
     */
    void finishTraining();

private:
    cv::BOWKMeansTrainer trainer;
};

#endif // BOW_DATABASE_H
