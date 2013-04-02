#ifndef CONFIGURATED_TOOLS_H
#define CONFIGURATED_TOOLS_H

/// COMPONENT
#include "config.h"

/// PROJECT
#include <common/global.hpp>
#include <utils/extractor.h>
#include <utils/matcher.h>
#include <utils/match_scorer.h>

/// SYSTEM
#include <boost/thread.hpp>
#include <string>

/**
 * @brief The ConfiguratedTools class contains different classes that depend
 *        on the most recent Config and are used by different parts of the program
 */
class ConfiguratedTools
{
public:
    typedef boost::shared_ptr<ConfiguratedTools> Ptr;

private:
    /**
     * @brief ConfiguratedTools
     * @param initial config
     */
    ConfiguratedTools(const Config& initial);

public:
    /**
     * @brief ~ConfiguratedTools
     */
    virtual ~ConfiguratedTools();

    /**
     * @brief create a new instance of the set of tools
     * @param initial config to use for creating the tools
     * @return
     */
    static Ptr create(const Config& initial);

public:
    Extractor::Ptr getExtractor();
    Matcher::Ptr  getMatcher();
    MatchScorer::Ptr getMatchScorer();

private:
//    boost::mutex mutex;

//    bool initialized;
//    boost::condition_variable initialized_cond;

    Extractor::Ptr  extractor;
    Matcher::Ptr  matcher;
    MatchScorer::Ptr  scorer;


    static ConfiguratedTools::Ptr tool_cache;
    static std::string tool_chache_hash;
};

#endif // CONFIGURATED_TOOLS_H
