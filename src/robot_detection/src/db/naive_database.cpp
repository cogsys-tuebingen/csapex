/// HEADER
#include "naive_database.h"

/// PROJECT
#include <data/matchable_pose.h>
#include <utils/match_scorer.h>


NaiveDatabase::NaiveDatabase()
{
}

void NaiveDatabase::add(MatchablePose* pose)
{
    poses.push_back(pose);

    changed();
}

void NaiveDatabase::traversePoses(boost::function<void (int, const std::string&)> composite_callback,
                                  boost::function<void (int, MatchablePose*)> leaf_callback)
{
    composite_callback(0, "naive");

    for(unsigned i = 0, n = poses.size(); i < n; ++i) {
        leaf_callback(1, poses[i]);
    }
}

void NaiveDatabase::finishTraining()
{

}

void NaiveDatabase::deletePoseByIndex(const int index)
{
    delete poses.at(index);
    poses.erase(poses.begin() + index);

    changed();
}

double NaiveDatabase::findBestMatch(Matchable* current_frame, MatchablePose *&out, int* no_of_features) const
{
    double best_score = INFINITY;
    unsigned n = poses.size();

    for(unsigned i = 0; i < n; ++i) {
        MatchablePose* pose = poses[i];

        int features = 0;
        double score = tools->getMatchScorer()->calculateScore(*current_frame, *pose, &features);

        if(score < best_score) {
            best_score = score;
            out = pose;
            if(no_of_features) {
                *no_of_features = features;
            }
        }
    }

    return best_score;
}

void NaiveDatabase::clear()
{
    poses.clear();
}

