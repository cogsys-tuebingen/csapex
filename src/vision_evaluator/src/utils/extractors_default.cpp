#ifndef EXTRACTORS_DEFAULT_HPP
#define EXTRACTORS_DEFAULT_HPP

/// COMPONENT
#include "extractor_manager.h"

/// SYSTEM
#include <opencv2/nonfree/nonfree.hpp>

using namespace csapex;

/// COMBINATIONS

struct Orb : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    static void keypoint(Extractor* e, bool complete) {
        Config config = Config::getGlobal();

        e->keypoint = config.getKeypointType();
        e->has_orientation = true;
        e->detector = new cv::ORB((200-config.extractor_threshold)*10, 1.2f, 8, 0, 12, 2, cv::ORB::FAST_SCORE, 31);

        if(complete) {
            e->is_binary = true;
            e->descriptor = config.getDescriptorType();
            e->descriptor_extractor = e->detector;
        }
    }

    static void descriptor(Extractor* e) {
        Config config = Config::getGlobal();

        e->is_binary = true;
        e->descriptor = config.getDescriptorType();
        e->descriptor_extractor = new cv::ORB();
    }
};
REGISTER_FEATURE_DETECTOR(Orb, ORB);
BOOST_STATIC_ASSERT(DetectorTraits<Orb>::HasKeypoint);
BOOST_STATIC_ASSERT(DetectorTraits<Orb>::HasDescriptor);

struct Brisk : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    static void keypoint(Extractor* e, bool complete) {
        Config config = Config::getGlobal();

        e->keypoint = config.getKeypointType();
        e->has_orientation = true;
        e->detector = new cv::BRISK(config.extractor_threshold, config.octaves, 2.0f);

        if(complete) {
            e->is_binary = true;
            e->descriptor = config.getDescriptorType();
            e->descriptor_extractor = e->detector;
        }
    }

    static void descriptor(Extractor* e) {
        Config config = Config::getGlobal();

        e->is_binary = true;
        e->descriptor = config.getDescriptorType();
        e->descriptor_extractor = new cv::BRISK(config.extractor_threshold, config.octaves);
    }

};
REGISTER_FEATURE_DETECTOR(Brisk, BRISK);
BOOST_STATIC_ASSERT(DetectorTraits<Brisk>::HasKeypoint);
BOOST_STATIC_ASSERT(DetectorTraits<Brisk>::HasDescriptor);


struct Sift : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    static void keypoint(Extractor* e, bool complete) {
        Config config = Config::getGlobal();

        e->keypoint = config.getKeypointType();
        e->has_orientation = true;
        e->detector = new cv::SiftFeatureDetector(config.extractor_threshold * 0.002 / 3, 10.0);

        if(complete) {
            e->is_binary = false;
            e->descriptor = config.getDescriptorType();
            e->descriptor_extractor = e->detector;
        }
    }

    static void descriptor(Extractor* e) {
        Config config = Config::getGlobal();

        e->is_binary = false;
        e->descriptor = config.getDescriptorType();
        e->descriptor_extractor = new cv::SiftDescriptorExtractor();
    }

};
REGISTER_FEATURE_DETECTOR(Sift, SIFT);
BOOST_STATIC_ASSERT(DetectorTraits<Sift>::HasKeypoint);
BOOST_STATIC_ASSERT(DetectorTraits<Sift>::HasDescriptor);



struct Surf : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    static void keypoint(Extractor* e, bool complete) {
        Config config = Config::getGlobal();

        e->keypoint = config.getKeypointType();
        e->has_orientation = true;
        e->detector = new cv::SurfFeatureDetector(config.extractor_threshold);

        if(complete) {
            e->is_binary = false;
            e->descriptor = config.getDescriptorType();
            e->descriptor_extractor = e->detector;
        }
    }

    static void descriptor(Extractor* e) {
        Config config = Config::getGlobal();

        e->is_binary = false;
        e->descriptor = config.getDescriptorType();
        e->descriptor_extractor = new cv::SurfDescriptorExtractor();
    }

};
REGISTER_FEATURE_DETECTOR(Surf, SURF);
BOOST_STATIC_ASSERT(DetectorTraits<Surf>::HasKeypoint);
BOOST_STATIC_ASSERT(DetectorTraits<Surf>::HasDescriptor);




/// KEYPOINTS ONLY


struct Fast : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    static void keypoint(Extractor* e, bool complete) {
        Config config = Config::getGlobal();

        e->keypoint = config.getKeypointType();
        e->has_orientation = false;
        e->detector = new cv::FastFeatureDetector(config.extractor_threshold, true);
    }
};
REGISTER_FEATURE_DETECTOR(Fast, FAST);
BOOST_STATIC_ASSERT(DetectorTraits<Fast>::HasKeypoint);
BOOST_STATIC_ASSERT(!DetectorTraits<Fast>::HasDescriptor);



struct Mser : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    static void keypoint(Extractor* e, bool complete) {
        Config config = Config::getGlobal();

        e->keypoint = config.getKeypointType();
        e->has_orientation = true;
        e->detector = new cv::MSER();
    }
};
REGISTER_FEATURE_DETECTOR(Mser, MSER);
BOOST_STATIC_ASSERT(DetectorTraits<Mser>::HasKeypoint);
BOOST_STATIC_ASSERT(!DetectorTraits<Mser>::HasDescriptor);



struct Star : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    static void keypoint(Extractor* e, bool complete) {
        Config config = Config::getGlobal();

        e->keypoint = config.getKeypointType();
        e->has_orientation = true;
        e->detector = new cv::StarFeatureDetector(16, config.extractor_threshold);
    }
};
REGISTER_FEATURE_DETECTOR(Star, STAR);
BOOST_STATIC_ASSERT(DetectorTraits<Star>::HasKeypoint);
BOOST_STATIC_ASSERT(!DetectorTraits<Star>::HasDescriptor);


struct Gftt : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    static void keypoint(Extractor* e, bool complete) {
        Config config = Config::getGlobal();

        e->keypoint = config.getKeypointType();
        e->has_orientation = true;
        e->detector = new cv::GoodFeaturesToTrackDetector(config.extractor_threshold * 50.0, 0.005);
    }
};
REGISTER_FEATURE_DETECTOR(Gftt, GFTT);
BOOST_STATIC_ASSERT(DetectorTraits<Gftt>::HasKeypoint);
BOOST_STATIC_ASSERT(!DetectorTraits<Gftt>::HasDescriptor);


struct GfttHarris : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    static void keypoint(Extractor* e, bool complete) {
        Config config = Config::getGlobal();

        e->keypoint = config.getKeypointType();
        e->has_orientation = true;
        e->detector = new cv::GoodFeaturesToTrackDetector(config.extractor_threshold * 50.0, 0.005, 2, 3, true);
    }
};
REGISTER_FEATURE_DETECTOR(GfttHarris, GFTT_HARRIS);
BOOST_STATIC_ASSERT(DetectorTraits<GfttHarris>::HasKeypoint);
BOOST_STATIC_ASSERT(!DetectorTraits<GfttHarris>::HasDescriptor);



/// DESCRIPTORS ONLY

struct Brief : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    static void descriptor(Extractor* e) {
        Config config = Config::getGlobal();

        e->is_binary = true;
        e->descriptor = config.getDescriptorType();
        e->descriptor_extractor = new cv::BriefDescriptorExtractor(64);
    }
};
REGISTER_FEATURE_DETECTOR(Brief, BRIEF);
BOOST_STATIC_ASSERT(!DetectorTraits<Brief>::HasKeypoint);
BOOST_STATIC_ASSERT(DetectorTraits<Brief>::HasDescriptor);


struct Freak : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    static void descriptor(Extractor* e) {
        Config config = Config::getGlobal();

        e->is_binary = true;
        e->descriptor = config.getDescriptorType();
        e->descriptor_extractor = new cv::FREAK();
    }
};
REGISTER_FEATURE_DETECTOR(Freak, FREAK);
BOOST_STATIC_ASSERT(!DetectorTraits<Freak>::HasKeypoint);
BOOST_STATIC_ASSERT(DetectorTraits<Freak>::HasDescriptor);

#endif // EXTRACTORS_DEFAULT_HPP
