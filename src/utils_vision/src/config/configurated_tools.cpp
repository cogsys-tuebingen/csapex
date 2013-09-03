/// HEADER
#include "configurated_tools.h"

/// COMPONENT
#include "config.h"

/// PROJECT
#include <utils/extractor_factory.h>
#include <utils/match_scorer_factory.h>

ConfiguratedTools::Ptr ConfiguratedTools::tool_cache;
std::string ConfiguratedTools::tool_chache_hash;

ConfiguratedTools::ConfiguratedTools(const Config& config)
    : extractor(ExtractorFactory::create(config("keypointType"), config("descriptorType"), Config::instance())),
      matcher(new Matcher(extractor->binary())), scorer(MatchScorerFactory::create(*matcher))
{
    assert(extractor);
    assert(matcher);
    assert(scorer);
}

ConfiguratedTools::Ptr ConfiguratedTools::create(const Config& config)
{
    std::string hash = config.computeHash();
    bool already_there = tool_chache_hash == hash;
    bool empty = tool_cache.get() == NULL;
    bool create = empty || !already_there;

    if(create) {
        try {
            ConfiguratedTools::Ptr tmp(new ConfiguratedTools(config));
            tool_cache = tmp;
            tool_chache_hash = hash;

        } catch(std::exception& e) {
            ERROR("failed to construct tools: " << e.what());
            std::abort();
        }
    }

    return tool_cache;
}

ConfiguratedTools::~ConfiguratedTools()
{
}

Extractor::Ptr ConfiguratedTools::getExtractor()
{
//    boost::mutex::scoped_lock(mutex);
//    while(!extractor){
//        initialized_cond.wait(mutex);
//    }
//    assert(extractor);
    return extractor;
}

Matcher::Ptr ConfiguratedTools::getMatcher()
{
//    boost::mutex::scoped_lock(mutex);
//    while(!matcher){
//        initialized_cond.wait(mutex);
//    }
//    assert(matcher);
    return matcher;
}

MatchScorer::Ptr ConfiguratedTools::getMatchScorer()
{
//    boost::mutex::scoped_lock(mutex);
//    while(!scorer){
//        initialized_cond.wait(mutex);
//    }
//    assert(scorer);
    return scorer;
}

