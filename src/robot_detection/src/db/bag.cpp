/// HEADER
#include "bag.h"

/// PROJECT
#include <config/config.h>
#include <data/matchable_pose.h>
#include <data/matchable.h>
#include <utils/extractor_factory.h>

/// SYSTEM
#include <fstream>

Bag::Bag()
{
}

Bag* Bag::create()
{
    const Config cfg = Config::instance();

    if(cfg("descriptorType").as<std::string>() == "ORB") {
        return new BagImplementation<DBoW2::FOrb::TDescriptor, DBoW2::FOrb>();

    } else {
        ERROR("descriptor type " << cfg("descriptorType").as<std::string>() << " not supported with bag database...");
        throw Extractor::IllegalDescriptorException();
    }

    assert(false);
    return NULL; // cannot be reached
}

void Bag::add(MatchablePose* pose)
{
    //    INFO( pose->descriptors.rows );
    //    cv::Mat img;
    //    cv::drawKeypoints(pose->image, pose->keypoints, img, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    //    cv::imshow("debug", img);
    //    cv::waitKey(33);
    poses.push_back(pose);
}

namespace
{
template<class TDescriptor>
void convert(const cv::Mat& descriptors, std::vector<TDescriptor>& out)
{
    ERROR("not implemented");
    throw Extractor::IllegalDescriptorException();
}

template<>
void convert(const cv::Mat& descriptors, std::vector<DBoW2::FOrb::TDescriptor>& out)
{
    out.resize(descriptors.rows);
    for(int r = 0; r < descriptors.rows; ++r) {
        const cv::Mat& row = descriptors.row(r);
        assert(row.type() == CV_8UC1);
        DBoW2::FOrb::fromMat8UC(out[r],row);
    }
}
}

template<class TDescriptor, class F>
void BagImplementation<TDescriptor, F>::poses2features(const std::vector<MatchablePose*> &poses, std::vector<std::vector<TDescriptor> > &features)
{
    features.clear();
    features.reserve(poses.size());

    for(std::vector<MatchablePose*>::const_iterator it = poses.begin(); it != poses.end(); ++it) {
        std::vector<TDescriptor> current;
        MatchablePose* pose = *it;
        convert<TDescriptor>(pose->descriptors, current);
        features.push_back(current);
    }
}

template<class TDescriptor, class F>
void BagImplementation<TDescriptor, F>::matchable2feature(const Matchable* matchable, std::vector<TDescriptor> &feature)
{
    feature.clear();
    convert<TDescriptor>(matchable->descriptors, feature);
}

template<class TDescriptor, class F>
void BagImplementation<TDescriptor, F>::train()
{
    const int k = 6;
    const int L = 3;
    const DBoW2::WeightingType weight = DBoW2::TF_IDF;
    const DBoW2::ScoringType score = DBoW2::L1_NORM;

    std::vector<std::vector<TDescriptor> > features;

    INFO("converting poses to features");
    poses2features(poses, features);

    INFO("creating vobabulary");
    voc.create(features, k, L, weight, score);
    INFO("vocabulary has effective levels: " << voc.getEffectiveLevels());

    INFO("creating database");
    db.setVocabulary(voc, true);

    INFO("adding " << features.size() << " database entries");
    for(unsigned i = 0, n = features.size(); i < n; ++i) {
        DBoW2::EntryId entry = db.add(features[i]);
        map[entry] = poses[i];
    }
}

template<class TDescriptor, class F>
double BagImplementation<TDescriptor, F>::query(Matchable* current_frame, MatchablePose *&out)
{
    DBoW2::QueryResults ret;
    std::vector<TDescriptor> feature;
    DBoW2::BowVector vec;

    matchable2feature(current_frame, feature);
    voc.transform(feature, vec);
    db.query(vec, ret, 5);

    INFO(ret);

    if(ret.empty()) {
        out = MatchablePose::NULL_POSE;
        return INFINITY;
    }

    out = map[ret[0].Id];

    /***
     * DBoW uses scores [0..1] where 1.0 is optimal
     * therefore we must convert them so that 0.0 is optimal
     */
    return 1.0 - ret[0].Score;
    //return 0;
}

template<class TDescriptor, class F>
BagImplementation<TDescriptor, F>::BagImplementation()
{
}
