/// HEADER
#include "extractor_factory.h"

/// SYSTEM
#include <opencv2/nonfree/nonfree.hpp>

#define USE_THRESHOLD_ADAPTATION 0

namespace
{
template<int a, int b>
double relative_threshold(int t)
{
    return a + std::min(100, std::max(0, t)) / 100.0 * (b - a);
}
}

ExtractorFactory::ExtractorFactory()
{
}

Extractor::Ptr ExtractorFactory::create(Types::Keypoint::ID keypoint, Types::Descriptor::ID descriptor)
{
    Extractor::Ptr e(new Extractor);
    init_keypoint(e, keypoint);
    init_descriptor(e, descriptor);
    return e;
}

void ExtractorFactory::init_keypoint(Extractor::Ptr& e, Types::Keypoint::ID keypoint)
{
    Config config = Config::getGlobal();

    int extractor_threshold = config.extractor_threshold;
    if(extractor_threshold == -1) extractor_threshold = 50;

    // 0 <= extractor_threshold <= 100

    int t = extractor_threshold;

    e->keypoint = config.getKeypointType();
    e->has_orientation = true;

    // detector
    // the relative thresholds were determined by the programm "thresh_norm" and then manually tweaked
    switch(keypoint) {
    case Types::Keypoint::BRISK:
#if USE_THRESHOLD_ADAPTATION
        t = relative_threshold<25, 93>(extractor_threshold);
#endif
        e->detector = new cv::BRISK(t, config.octaves, 2.0f);
        break;
    case Types::Keypoint::SIFT: {
#if USE_THRESHOLD_ADAPTATION
        double td = relative_threshold<17, 237>(extractor_threshold);
#else
        double td = extractor_threshold;
#endif
        td *= 0.002 / 3;

        e->detector = new cv::SiftFeatureDetector(td, 10.0);
    }
    break;
    case Types::Keypoint::SURF:
#if USE_THRESHOLD_ADAPTATION
        t = relative_threshold<100, 3000>(extractor_threshold);
#else
#endif
        e->detector = new cv::SurfFeatureDetector(t);
        break;
    case Types::Keypoint::ORB:
#if USE_THRESHOLD_ADAPTATION
        t = relative_threshold<9, 81>(extractor_threshold);
#endif
        e->detector = new cv::ORB((200-t)*10, 1.2f, 8, 0, 12, 2, cv::ORB::FAST_SCORE, 31);
        break;
    case Types::Keypoint::FAST:
#if USE_THRESHOLD_ADAPTATION
        t = relative_threshold<9, 81>(extractor_threshold);
#endif
        e->detector = new cv::FastFeatureDetector(t, true);
        e->has_orientation = false;
        break;
    case Types::Keypoint::AGAST:
#if USE_THRESHOLD_ADAPTATION
        t = relative_threshold<9, 81>(extractor_threshold);
#endif
#if OLD_BRISK
        e->detector = new cv::BriskFeatureDetector(t, 0);
#else
        e->detector = new cv::BRISK(t, config.octaves);
#endif
        break;
    case Types::Keypoint::MSER:
#if USE_THRESHOLD_ADAPTATION
        t = relative_threshold<1, 12>(extractor_threshold);
#endif
#if CV_MAJOR_VERSION >= 2 && CV_MINOR_VERSION >= 4
//        e->detector = new cv::MSER(45, t);
        e->detector = new cv::MSER();
#else
        e->detector = new cv::MserFeatureDetector(cvMSERParams(t));
#endif
        break;
    case Types::Keypoint::STAR:
#if USE_THRESHOLD_ADAPTATION
        t = relative_threshold<1, 49>(extractor_threshold);
#endif
        e->detector = new cv::StarFeatureDetector(16, t);
        break;
    case Types::Keypoint::GFTT: {
#if USE_THRESHOLD_ADAPTATION
        t = relative_threshold<Extractor::TARGET_MAX_FEATURE_COUNT / 10, 1>(extractor_threshold);
#else
        t = extractor_threshold * 50.0;
#endif
        e->detector = new cv::GoodFeaturesToTrackDetector(t, 0.005, 2);
    }
    break;
    default:
        ERROR("Keypoint " << config.getKeypointType() << " not recognized. Check spelling!");
        throw Types::Keypoint::IllegalException();
    }

    assert(!e->detector.empty());

    INFO("new detector: " << Types::Keypoint::write(config.getKeypointType()));
}

void  ExtractorFactory::init_descriptor(Extractor::Ptr& e, Types::Descriptor::ID descriptor)
{
    Config config = Config::getGlobal();

    e->is_binary = true;

    e->descriptor = config.getDescriptorType();

    switch(descriptor) {
    case Types::Descriptor::BRISK:
        e->descriptor_extractor = new cv::BRISK(config.extractor_threshold, config.octaves);
        break;
    case Types::Descriptor::BRIEF:
        e->descriptor_extractor = new cv::BriefDescriptorExtractor(64);
        break;
    case Types::Descriptor::ORB:
        e->descriptor_extractor = new cv::ORB();
        break;
    case Types::Descriptor::SURF:
        e->descriptor_extractor = new cv::SurfDescriptorExtractor();
        e->is_binary = false;
        break;
    case Types::Descriptor::SIFT:
        e->descriptor_extractor = new cv::SiftDescriptorExtractor();
        e->is_binary = false;
        break;
    case Types::Descriptor::FREAK:
        e->descriptor_extractor = new cv::FREAK();
        break;
    default:
        ERROR("Descriptor " << config.getDescriptorType() << " not recognized. Check spelling!");
        throw Types::Descriptor::IllegalException();
    }

    assert(!e->descriptor_extractor.empty());

    INFO("new descriptor: " << Types::Descriptor::write(config.getDescriptorType()));
}
