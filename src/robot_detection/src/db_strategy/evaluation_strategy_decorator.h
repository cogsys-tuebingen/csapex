#ifndef BIN_EVAL_DB_STRATEGY_H
#define BIN_EVAL_DB_STRATEGY_H

/// COMPONENT
#include "db_strategy_decorator.h"

/// PROJECT
#include <roc/roc_creator.h>

/**
 * @brief The BinEvaluationDatabaseStrategy class is used to generate statistics
 */
class EvaluationStrategyDecorator : public DatabaseStrategyDecorator
{
public:
    typedef boost::shared_ptr<EvaluationStrategyDecorator> DecoPtr;

public:
    /**
     * @brief EvaluationStrategyDecorator
     * @param wrapped The Strategy to decorate
     */
    EvaluationStrategyDecorator(Ptr decorated);

    /**
     * @brief ~EvaluationStrategyDecorator
     */
    virtual ~EvaluationStrategyDecorator();

    /**
     * @brief testFrame Evaluate the database on an image
     * @param frame the frame to analyze
     * @param is_positive true, if the given frame is a positive example
     * @return false, iff shutdown requested
     */
    bool testFrame(Frame::Ptr frame, bool is_positive);

    /**
     * @brief outputTestResults tests the database using test examples
     * @param out outstream for results
     * @return
     */
    virtual bool outputTestResults(std::ostream& out);


    /**
     * @brief setThresholdScoreCB Callback function for the RocCreator
     * @param roc_creator caller
     * @param score selected score
     * @param save save the db?
     * @param out outstream for results
     */
    void setThresholdScoreCB(RocCreator* roc_creator, const RocCreator::Data* score, bool save, std::ostream* out =NULL);

private:
    void startEvaluation();
    MatchablePose* test(Frame::Ptr current, RocCreator::Data& rocdata, bool positive);

private:
    int p;
    int n;

    int tp;
    int fp;
    int tn;
    int fn;

    double e_r;
    double e_theta;

    std::vector<int> extract_time;
    std::vector<int> lookup_time;
    std::vector<int> feature_count_frame;
    std::vector<int> feature_count_robot;

    std::vector<double> error_theta;

    long summed_extract_time;
    long summed_lookup_time;
    long summed_feature_count;

    RocCreator* roc_creator;

    bool repeat_evaluation;
};

#endif // BIN_EVAL_DB_STRATEGY_H
