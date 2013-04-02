#ifndef ROBOT_HYPOTHESIS_H
#define ROBOT_HYPOTHESIS_H

/// SYSTEM
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <vector>

using namespace Eigen;

/**
 * @brief The Hypothesis class represents one hypothesis of a robotpose
 */
class Hypothesis
{
    friend class Filter;

public:
    /**
     * @brief NULL_HYPOTHESIS is returned whenever no pose was found
     */
    static Hypothesis* NULL_HYPOTHESIS;

public:
    /**
     * @brief Hypothesis
     * @param x mean
     * @param P covariance
     * @param decay_time time until the hypothesis will be deleted
     * @param decay_per_update time that gets added to the lifeteam on each update
     */
    Hypothesis(const Eigen::VectorXd& x, const Eigen::MatrixXd& P, double decay_time, double decay_per_update);

    /**
     * @brief predict predicts this hypothesis for the next timestep
     * @param dt timedifference
     * @return true, iff the hypothesis should be kept
     */
    bool predict(double dt);

    /**
     * @brief update updates the hypothesis
     * @param measurement the measurement to update with
     */
    void update(const Eigen::Vector3d& measurement);

    /**
     * @brief getState Accessor
     * @return the state
     */
    VectorXd getState() const {
        return VectorXd(x);
    }

    /**
     * @brief getCovariance Accessor
     * @return the covariance
     */
    MatrixXd getCovariance() const {
        return MatrixXd(P);
    }

    /**
     * @brief getId Accessor
     * @return the id
     */
    long getId() const {
        return id;
    }

    /**
     * @brief getNoTimesUpdated Accessor
     * @return
     */
    int getNoTimesUpdated() const {
        return no_times_updated;
    }

    /**
     * @brief getDecay Accessor
     * @return lifetime left
     */
    double getDecay() const {
        return decay;
    }

    /**
     * @brief probability calculates the value of the probability density at the given point
     * @param pose point to calculate at
     * @return the probability density
     */
    double probability(const Eigen::Vector3d& pose);

private:
    Eigen::MatrixXd makeF(double dt);

private:
    static Eigen::MatrixXd Q; // noise_process
    static Eigen::MatrixXd R; // noise_measurement;

    static Eigen::MatrixXd H; // maps state space to measured space;
    static Eigen::MatrixXd I;

    static bool initializeConstants();
    static bool initialized;

private:
    VectorXd x;
    MatrixXd P;

    std::vector<int> angular_components_;

    double decay_max;
    double decay_step;
    double decay;

    int no_times_updated;

    static long next_id;
    long id;
};

#endif // ROBOT_HYPOTHESIS_H
