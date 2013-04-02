#ifndef MATCH_SCORER_FACTORY_H
#define MATCH_SCORER_FACTORY_H

/// COMPONENT
#include "match_scorer.h"

/// SYSTEM
#include <exception>

/**
 * @brief The MatchScorerFactory class creates a MatchScorer
 */
class MatchScorerFactory
{
public:
    class IllegalScorerException : public std::exception {};

    /**
     * @brief The Scorer class represents symbolic names for the different scorers
     */
    class Scorer
    {
    public:
        enum Type {
            HOMOGRAPHY, CLUSTERING, REPROJECTION,
            COUNT
        };
    };

private:
    /**
     * @brief MatchScorerFactory
     */
    MatchScorerFactory();

public:
    /**
     * @brief create creates a new Scorer
     * @param matcher the matcher to use for scoring
     * @return the new Scorer, responsibility for deleting given to caller
     */
    static MatchScorer::Ptr create(Matcher& matcher);

    /**
     * @brief setType sets the type of match scorer to generate next
     * @param type
     */
    static void setType(Scorer::Type type);

protected:
    static Scorer::Type create_type;
};

#endif // MATCH_SCORER_FACTORY_H
