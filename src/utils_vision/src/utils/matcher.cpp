/// HEADER
#include "matcher.h"

/// PROJECT
#include <config/config.h>
#include <data/matchable.h>


Matcher::Matcher(bool binary)
    : hamming(binary), threshold(0.8)
{
    if(binary) {
#if CV_MAJOR_VERSION <= 2 && CV_MINOR_VERSION < 4
        descriptor_matcher = new cv::BruteForceMatcher<cv::HammingSse>();
#else
        descriptor_matcher = new cv::BFMatcher(cv::NORM_HAMMING);
#endif
    } else {
        descriptor_matcher = new cv::FlannBasedMatcher();
    }

    const Config config = Config::getGlobal();

    min_points = config.min_points;
    threshold = config.matcher_threshold;
}

double Matcher::matchFiltered(Matchable* a, const Matchable* b,
                              std::vector<cv::KeyPoint> &filtered_a_keypoints,
                              std::vector<cv::KeyPoint> &filtered_b_keypoints,
                              std::vector<std::vector<cv::DMatch> > * match_out) const
{

    filtered_a_keypoints.clear();
    filtered_b_keypoints.clear();

    std::vector<std::vector<cv::DMatch> > &matches = a->last_matches;
    matches.clear();

    match(a, b, matches);

    double min_dist = 200;
    double max_dist = -INFINITY;

    for(int i=0, n = matches.size(); i < n; ++i) {
        if(matches[i].empty())
            continue;

        double dist = matches[i][0].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }

    for(int i=0, n = matches.size(); i < n; ++i) {
        if(matches[i].empty())
            continue;

        if(matches[i].size() >= 2) {
            double ratio = matches[i][0].distance / matches[i][1].distance;
            if(ratio > 0.8) {
                continue;
            }
        }

        cv::DMatch& m = matches[i][0];
        if(m.distance < 2.0 * min_dist) {
            filtered_a_keypoints.push_back(a->keypoints[m.queryIdx]);
            filtered_b_keypoints.push_back(b->keypoints[m.trainIdx]);
        } else {
            matches[i].clear();
        }
    }

    if(match_out) {
        match_out->insert(match_out->begin(), matches.begin(), matches.end());
    }

    double ratio = filtered_a_keypoints.size() / (double) a->keypoints.size();

    return ratio;
}

void Matcher::match(const Matchable* a, const Matchable* b, std::vector<std::vector<cv::DMatch> > &out) const
{
    int k = std::min(4, std::min(a->descriptors.rows-1, b->descriptors.rows-1));

    if(k > 0) {
        if(hamming) {
            //descriptor_matcher->radiusMatch(a->descriptors, b->descriptors, out, (double) dist);
            descriptor_matcher->knnMatch(a->descriptors, b->descriptors, out, k);
        } else {
            //        descriptor_matcher->radiusMatch(a->descriptors, b->descriptors, out, dist / 100.0);
            descriptor_matcher->knnMatch(a->descriptors, b->descriptors, out, k);
        }
    }

    /// SIFT:
    /// To increase robustness, matches are rejected for those keypoints for which the
    /// ratio of the nearest neighbor distance to the second nearest
    /// neighbor distance is greater than 0.8. This discards many of the
    /// false matches arising from background clutter.
    for(std::vector<std::vector<cv::DMatch> >::iterator it = out.begin(); it != out.end();) {
        std::vector<cv::DMatch> &matches = *it;
        if(matches.size() <= 1) {
            it = out.erase(it);

        } else {
            double ratio = matches[0].distance / matches[1].distance;
            if(ratio > threshold) {
                it = out.erase(it);
            } else {
                ++it;
                // here we drop all matches that are "worse"
                // TODO: somehow also use the other matches, if they are not "too bad"
                while(matches.size() > 1) {
                    matches.pop_back();
                }
            }
        }
    }
}

namespace
{
template <typename T>
void anti_filter(Matchable* positive, std::deque<unsigned> &delete_indices)
{
    unsigned to_be_removed = delete_indices.size();
    unsigned orig_size = positive->keypoints.size();
    unsigned new_size = orig_size - to_be_removed;
    unsigned cols = positive->descriptors.cols;
    int type = positive->descriptors.type();

    std::vector<cv::KeyPoint> keypoints_output;
    keypoints_output.reserve(new_size);
    cv::Mat descriptors_output(new_size, cols, type);


    // sort the indices asscending
    std::sort(delete_indices.begin(), delete_indices.end());

    // iterate the original data and copy rows that should not be deleted
    unsigned row_output = 0;
    for(unsigned row_input = 0; row_input < orig_size; ++row_input) {
        if(!delete_indices.empty() && delete_indices.front() == row_input) {
            // this row should be deleted
            // remove this index from the deque
            delete_indices.pop_front();
        } else {
            // this row should be kept
            keypoints_output.push_back(positive->keypoints[row_input]);
            for(unsigned col = 0; col < cols; ++col) {
                descriptors_output.at<T>(row_output, col) = positive->descriptors.at<T>(row_input, col);
            }
            row_output++;
        }
    }

    assert(row_output == new_size);

    // copy back the new data
    positive->keypoints = keypoints_output;
    positive->descriptors = descriptors_output;
}
}

void Matcher::matchAntiFilter(Matchable* positive, const Matchable* negative) const
{

    std::vector<std::vector<cv::DMatch> > matches;

    match(positive, negative, matches);

    // find all indices to be deleted
    std::deque<unsigned> delete_indices;
    for(int i=0, n = matches.size(); i < n; ++i) {
        if(matches[i].empty()) {
            // doesn't match a negative example -> don't delete
            continue;
        }
        cv::DMatch& m = matches[i][0];

        delete_indices.push_back(m.queryIdx);
    }

    unsigned to_be_removed = delete_indices.size();
    if(to_be_removed > 0) {

        int type = positive->descriptors.type();
        assert(type == CV_8U || type == CV_32F);

        if(type ==CV_8U) {
            anti_filter<unsigned char>(positive, delete_indices);
        } else {
            anti_filter<float>(positive, delete_indices);
        }
    }
}
