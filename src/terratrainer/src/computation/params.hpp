#ifndef PARAMS_HPP
#define PARAMS_HPP
/// SYSTEM
#include <vector>
#include <yaml-cpp/yaml.h>
#include "yaml.hpp"

#define CV_TERMCRIT_ITER 1
#define CV_TERMCRIT_EPS  2

struct CMPForestParams {
    /**
     *CV_TERMCRIT_ITER = 1;
     *CV_TERMCRIT_EPS  = 2;
     */

    CMPForestParams() :
        max_depth(25), min_samples(5), regression(0),
        surrogates(false), max_categories(15),
        variable_importance(false),
        nactive_variables(4), max_trees(100),
        accurracy(0.01f),
        termination_criteria(CV_TERMCRIT_ITER |	CV_TERMCRIT_EPS),
        dirty(true){fillPriors();}

    int                max_depth;
    int                min_samples;
    float              regression;
    bool               surrogates;
    int                max_categories;
    std::vector<float> priors;
    bool               variable_importance;
    int                nactive_variables;
    int                max_trees;
    float              accurracy;
    int                termination_criteria;
    bool               dirty;

    void write(YAML::Emitter &emitter) const
    {
        emitter << YAML::Key << "FOREST" << YAML::Value;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "max_depth"             << YAML::Value << max_depth;
        emitter << YAML::Key << "min_samples"           << YAML::Value << min_samples;
        emitter << YAML::Key << "regression"            << YAML::Value << regression;
        emitter << YAML::Key << "max_categories"        << YAML::Value << max_categories;
        emitter << YAML::Key << "variable_importance"   << YAML::Value << variable_importance;
        emitter << YAML::Key << "nactive_variables"     << YAML::Value << nactive_variables;
        emitter << YAML::Key << "max_trees"             << YAML::Value << max_trees;
        emitter << YAML::Key << "accurracy"             << YAML::Value << accurracy;
        emitter << YAML::Key << "termination_criteria"  << YAML::Value << termination_criteria;
        emitter << YAML::EndMap;
    }

    void read(const YAML::Node &document)
    {
        try {
            const YAML::Node &data = document["FOREST"];
            data["max_depth"]            >> max_depth;
            data["min_samples"]          >> min_samples;
            data["regression"]           >> regression;
            data["max_categories"]       >> max_categories;
            data["variable_importance"]  >> variable_importance;
            data["nactive_variables"]    >> nactive_variables;
            data["max_trees"]            >> max_trees;
            data["accurracy"]            >> accurracy;
            data["termination_criteria"] >> termination_criteria;
            fillPriors();

        } catch (YAML::Exception e) {
            std::cerr << "ORB Parameters cannot read config : '" << e.what() <<"' !" << std::endl;
        }
    }

    void fillPriors() {
        priors.clear();
        for(int i = 0 ; i < max_categories ; ++i) {
            priors.push_back(1.f);
        }
    }
};

struct CMPGridParams {
    CMPGridParams() : cell_height(10), cell_width(10), dirty(true){}
    int    cell_height;
    int    cell_width;
    bool   dirty;

    void write(YAML::Emitter &emitter) const
    {
        emitter << YAML::Key << "GRID" << YAML::Value;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "cell_height" << YAML::Value << cell_height;
        emitter << YAML::Key << "cell_width"  << YAML::Value  << cell_width;
        emitter << YAML::EndMap;
    }
    void read(const YAML::Node &document)
    {
        try {

            const YAML::Node &data = document["GRID"];

            data["cell_height"] >> cell_height;
            data["cell_width"]  >> cell_width;

        } catch (YAML::Exception e) {
            std::cerr << "ORB Parameters cannot read config : '" << e.what() <<"' !" << std::endl;
        }
    }
};

struct CMPQuadParams {
    CMPQuadParams() : min_height(10), min_width(10), min_prob(0.75),dirty(true){}
    int     min_height;
    int     min_width;
    float   min_prob;
    bool    dirty;

    void write(YAML::Emitter &emitter) const
    {
        emitter << YAML::Key << "QUAD" << YAML::Value;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "min_height"    << YAML::Value << min_height;
        emitter << YAML::Key << "min_width"     << YAML::Value << min_width;
        emitter << YAML::Key << "min_prob"      << YAML::Value << min_prob;
        emitter << YAML::EndMap;
    }

    void read(const YAML::Node &document)
    {
        try {

            const YAML::Node &data = document["GRID"];

            data["min_height"] >> min_height;
            data["min_width"]  >> min_width;
            data["min_prob"]   >> min_prob;

        } catch (YAML::Exception e) {
            std::cerr << "ORB Parameters cannot read config : '" << e.what() <<"' !" << std::endl;
        }
    }
};

#endif // PARAMS_HPP
