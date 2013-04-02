/// HEADER
#include "bin_database.h"

/// PROJECT
#include <data/matchable_pose.h>
#include <utils/extractor.h>
#include <utils/matcher.h>
#include <utils/match_scorer.h>

/// SYSTEM
#include <angles/angles.h>
#include <boost/archive/polymorphic_iarchive.hpp>
#include <boost/archive/polymorphic_oarchive.hpp>

BinDatabase::BinDatabase()
    : bin_max_dist_(100)
{
    Config initial = Config::getGlobal();
    assert(initial.bin_count > 0);

    makeBins(bin_max_dist_);
}

void BinDatabase::makeBins(int max_dist)
{
    double slice = 2 * M_PI / config.bin_count;
    for(int i = 0; i < config.bin_count; ++i) {
        double angle_low = i * slice;
        double angle_high = (i+1) * slice;
        Bin b(angle_low, angle_high, config.bin_max_poses_count);

        bins.push_back(b);
    }
}

double BinDatabase::findBestMatch(Matchable* current_frame, MatchablePose *& out, int* no_of_features) const
{
    assert(tools->getMatcher());
    assert(tools->getMatchScorer());

    double best_score = INFINITY;

    std::vector<Bin>::const_iterator best_bin = bins.begin();

    for(std::vector<Bin>::const_iterator bin = bins.begin(); bin != bins.end(); ++bin) {
        MatchablePose* pose;
        int feat = 0;
        double score = bin->findBestMatch(current_frame, pose, &feat);

        assert(score >= 0.0);

        if(score < best_score) {
            best_bin = bin;
            best_score = score;
            out = pose;
            if(no_of_features) {
                *no_of_features = feat;
            }
        }
    }

    debug = best_bin->best_debug;

    return best_score;
}

void BinDatabase::traversePoses(boost::function<void (int, const std::string&) > composite_callback,
                                boost::function<void (int, MatchablePose*) > leaf_callback)
{
    for(std::vector<Bin>::iterator it = bins.begin(); it != bins.end(); ++it) {
        Bin& b = *it;
        std::stringstream ss;
        ss << "bin [" << b.angle_low << ", " << b.angle_high << "], size=" << b.poses.size();
        composite_callback(0, ss.str());

        for(unsigned i = 0, n = b.poses.size(); i < n; ++i) {
            leaf_callback(1, b.poses[i]);
        }
    }
}


int BinDatabase::angle2index(double angle) const
{
    assert(!std::isnan(angle));
    assert(!std::isinf(angle));

    double theta = angles::normalize_angle_positive(angle);
    // 0 <= theta <= 2 PI
    int index = round(theta / (2.0 * M_PI) * (config.bin_count - 1));

    assert(index >= 0);
    assert(index < config.bin_count);

    return index;
}

void BinDatabase::add(MatchablePose* pose)
{
    double yaw = pose->orientation.toRadians();
    if(std::isinf(yaw) || std::isnan(yaw)) {
        ERROR("cannot add pose, orientation is invalid");
        return;
    }

    int index = angle2index(yaw);

    bins[index].add(pose);

    changed();
}

void BinDatabase::addValidationExample(Matchable* m)
{
    double yaw = m->orientation.toRadians();
    if(std::isinf(yaw) || std::isnan(yaw)) {
        ERROR("cannot add validation pose, orientation is invalid");
        return;
    }
    bins[angle2index(yaw)].addValidationExample(m);
}

void BinDatabase::finishTraining()
{
    assert(tools->getMatcher());
    assert(tools->getMatchScorer());


//    for(std::vector<Bin>::iterator it = bins.begin(); it != bins.end(); ++it) {
//        it->dump(config.config_dir + "/export/");
//    }

    INFO("pruning bins:");
    int i = 0;
    for(std::vector<Bin>::iterator it = bins.begin(); it != bins.end(); ++it) {
        INFO("bin #" << i << ": ");
        INFO(" no of entries before pruning: " << it->size());

        it->filter();
//        it->merge();

        INFO(" no of entries after pruning: " << it->size());

        i++;
    }

    changed();
}

void BinDatabase::clear()
{
    bins.clear();
    makeBins(bin_max_dist_);
}

