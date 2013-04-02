/// HEADER
#include "bin.h"

/// PROJECT
#include <data/matchable_pose.h>
#include <data/frame.h>
#include <utils/matcher.h>
#include <utils/match_scorer.h>

/// SYSTEM
#include <boost/filesystem.hpp>

Bin::Bin(double angle_low, double angle_high, int max_pose_count)
    : angle_low(angle_low), angle_high(angle_high), max_pose_count(max_pose_count)
{
}

Bin::~Bin()
{

}

double Bin::findBestMatch(Matchable* current_frame, MatchablePose *& out, int* no_of_features) const
{
    assert(tools->getMatcher());
    assert(tools->getMatchScorer());

    double best_score = INFINITY;

    best_debug = cv::Mat();

    for(std::vector<MatchablePose*>::const_iterator pose = poses.begin(); pose != poses.end(); ++pose) {
        int feat = 0;
        double score = tools->getMatchScorer()->calculateScore(*current_frame, **pose, &feat);

        if(score < best_score) {
            best_score = score;
            out = *pose;

            best_debug = tools->getMatchScorer()->debug;

            if(no_of_features) {
                *no_of_features = feat;
            }
        }

        if(best_debug.empty()) {
            best_debug = tools->getMatchScorer()->debug;
        }
    }

    return best_score;
}

void Bin::add(MatchablePose* pose)
{
    if((int) pose->keypoints.size() < config.min_points) {
        return;
    }

    poses.push_back(pose);
    // normalize orientation to bin
    pose->orientation = (angle_low + angle_high) / 2.0;

    sortPosesByDistance();
}

void Bin::addValidationExample(Matchable* m)
{
    validation.push_back(m);
}

void Bin::setSaved(bool saved) const
{
    for(std::vector<MatchablePose*>::const_iterator it = poses.begin(); it != poses.end(); ++it) {
        (*it)->saved = saved;
    }
}

namespace
{
static bool moreFeatures(const MatchablePose* lhs, const MatchablePose* rhs)
{
    return lhs->keypoints.size() > rhs->keypoints.size();
}

}

void Bin::merge()
{
    int feature_count = 0;
    for(std::vector<MatchablePose*>::const_iterator it = poses.begin(); it != poses.end(); ++it) {
        const MatchablePose& m = **it;
        feature_count += m.keypoints.size();
    }

    if(feature_count == 0) {
        return;
    }

    MatchablePose* result = new MatchablePose();
    const MatchablePose& first_pose = **poses.begin();
    result->keypoints.resize(feature_count);
    result->descriptors = cv::Mat(feature_count, first_pose.descriptors.cols, first_pose.descriptors.type());
    result->distance = first_pose.distance;
    result->image = first_pose.image;
    result->orientation = first_pose.orientation;

    int row_result = 0;
    for(std::vector<MatchablePose*>::const_iterator it = poses.begin(); it != poses.end(); ++it) {
        const MatchablePose& m = **it;
        int n = m.keypoints.size();
        for(int row = 0; row < n; ++row) {
            result->keypoints[row_result] = m.keypoints[row];
            result->descriptors.row(row_result) = m.descriptors.row(row);
            row_result++;
        }
    }

    poses.clear();
    for(std::vector<MatchablePose*>::iterator it = poses.begin(); it != poses.end(); ++it) {
        MatchablePose* m = *it;
        delete m;
    }

    poses.push_back(result);

    WARN("pose count is now " << poses.size());
}

void Bin::filter()
{
    assert(tools->getMatcher());
    assert(tools->getMatchScorer());

    std::map<MatchablePose*, std::vector<MatchablePose*> > covers;

    // create working copy
    std::vector<MatchablePose*> candidates = poses;
    poses.clear();

    // build map
    for(int i = 0, n = candidates.size(); i < n; ++i) {
        MatchablePose* candidate = candidates[i];
        for(int j = i+1; j < n; ++j) {
            MatchablePose* compare = candidates[j];
            // check if candidate covers compare
            // if so, insert into map
            std::vector<cv::KeyPoint> filtered_candidate_keypoints;
            std::vector<cv::KeyPoint> filtered_compare_keypoints;

            tools->getMatcher()->matchFiltered(candidate, compare, filtered_candidate_keypoints, filtered_compare_keypoints);

            if((int) filtered_candidate_keypoints.size() >= 1.5 * config.min_points) {
                covers[candidate].push_back(compare);
                covers[compare].push_back(candidate);
            }
        }
    }

    // find non matched candidates
    bool ignore_lone = false;//candidates.size() > 4;

    int lone = 0;
    for(std::vector<MatchablePose*>::iterator candidate = candidates.begin(); candidate != candidates.end();) {
        if(covers[*candidate].empty()) {
            if(!ignore_lone) {
                // keep this candidate
                poses.push_back(*candidate);
            }
            covers.erase(*candidate);
            // remove from candidate list
            candidate = candidates.erase(candidate);
            lone++;
        } else {
            ++candidate;
        }
    }
    INFO("found " << lone << " lonely candidates");


    // sort the candidates by feature count
    std::sort(candidates.begin(), candidates.end(), moreFeatures);


    // if there are candidates left, there must be at least two of them
    // a single candidate could not have covered any other poses and would have been selected already
    assert(candidates.size() >= 2 || candidates.empty());

    // while there are candidates left, use the one that is covered the most (== covers the most others)
    std::vector<MatchablePose*> not_covered = candidates;
    while(!not_covered.empty() && (int) poses.size() < max_pose_count) {
        // use the first not covered pose as default
        MatchablePose* best = not_covered[0];

        // then search the pose that covers the most other poses
        if(not_covered.size() > 1) {
            // more than one left, greedy search
            for(unsigned i = 0; i < candidates.size(); ++i) {
                if(candidates[i] != best) {
                    if(covers[best] < covers[candidates[i]]) {
                        best = candidates[i];
                    }
                }
            }
        }

        // use the candidate
        poses.push_back(best);

        // update not_covered list, remove all poses that get covered by 'best'
        for(std::vector<MatchablePose*>::iterator it = covers[best].begin(); it != covers[best].end(); ++it) {
            std::vector<MatchablePose*>::iterator pos = std::find(not_covered.begin(), not_covered.end(), *it);
            if(pos != not_covered.end()) {
                not_covered.erase(pos);
            }
            // also remove from the candidate list
            pos = std::find(candidates.begin(), candidates.end(), *it);
            if(pos != candidates.end()) {
                candidates.erase(pos);
            }
        }

        // remove the candidate itself from the not_covered list, if it is there
        std::vector<MatchablePose*>::iterator pos = std::find(not_covered.begin(), not_covered.end(), best);
        if(pos != not_covered.end()) {
            not_covered.erase(pos);
        }

        // remove candidate from the candidate list
        pos = std::find(candidates.begin(), candidates.end(), best);
        // 'best' must be on the candidate list
        assert(pos != candidates.end());
        candidates.erase(pos);

        // remove all references to 'best' from the coverage map
        covers.erase(best);
        for(std::map<MatchablePose*, std::vector<MatchablePose*> >::iterator it = covers.begin(); it != covers.end(); ++it) {
            while(true) {
                std::vector<MatchablePose*>::iterator pos = std::find(it->second.begin(), it->second.end(), best);
                if(pos == it->second.end()) {
                    break;
                }
                it->second.erase(pos);
            };
        }
    }

    // delete left over candidates
    for(std::vector<MatchablePose*>::iterator candidate = candidates.begin(); candidate != candidates.end();) {
        delete *candidate;
        candidate = candidates.erase(candidate);
    }

    // sort by distance
    sortPosesByDistance();

    // calculate the matching distance, such that >= 90 % of validation images are matched
    //                                   and that max_bin_dist <= max_dist

//    max_bin_dist = matcher->max_distance();

    // measure on validation set
    // variance on the max_dist until tp rate is too low
    // @TODO: make parameter
//    int delta = 5;

//    double last_ratio = 1.0;
//    double best_ratio = last_ratio;
//    double best_dist = max_bin_dist;
//    int best_correct = 0;
//    int validation_count = validation.size();

//    while(validation_count > 0){
//        int correct = 0;

//        for(std::vector<Pose*>::iterator validate = validation.begin(); validate != validation.end(); validate++){
//            Pose * pose = *validate;
//            Pose * best_match = NULL;

//            find_best_pose(pose, best_match);

//            if(best_match){
//                correct++;
//            }
//        }

//        last_ratio = correct / (double) validation_count;

//        // @TODO: make parameter
//        if(last_ratio < 0.95 || max_bin_dist < 2 * delta){
//            // ratio now to small
//            break;

//        } else {
//            // ratio still ok
//            best_correct = correct;
//            best_ratio = last_ratio;
//            best_dist = max_bin_dist;

//            max_bin_dist -= delta;
//        }
//    }

//    max_bin_dist = best_dist;

    //INFO( "tp=" << best_correct << ", fn=" << (validation_count - best_correct) << ", best_ratio=" << best_ratio << ", max_dist=" << max_bin_dist );
}

void Bin::sortPosesByDistance()
{
    if(poses.empty()) {
        return;
    }

    double preferred_distance = 2.0;

    // move poses that are closer than the threshold to the back
    std::vector<MatchablePose*> smaller;
    std::vector<MatchablePose*> larger;

    for(std::vector<MatchablePose*>::iterator it = poses.begin(); it != poses.end(); ++it) {
        if((*it)->distance < preferred_distance) {
            smaller.push_back(*it);
        } else {
            larger.push_back(*it);
        }
    }

    // sort by distance
    std::sort(smaller.begin(), smaller.end(), MatchablePose::MoreThan());
    std::sort(larger.begin(), larger.end(), MatchablePose::LessThan());

    poses.clear();

    poses.insert(poses.end(), larger.begin(), larger.end());
    poses.insert(poses.end(), smaller.begin(), smaller.end());
}

int Bin::size() const
{
    return poses.size();
}

void Bin::dump(const std::string& path) const
{
    std::stringstream ss;
    ss << path << "/" << (angle_low + angle_high) / 2 << "/";
    std::string dir = ss.str();

    boost::filesystem3::create_directories(dir);

    for(std::vector<MatchablePose*>::const_iterator it = poses.begin(); it != poses.end(); ++it) {
        const MatchablePose& m = **it;
        int fileno = 0;
        std::stringstream file;
        file << dir << "/" << fileno << ".png";
        cv::imwrite(file.str(), m.image);
        fileno++;
    }
}
