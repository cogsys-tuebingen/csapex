#ifndef EXTRACTORS_DEFAULT_HPP
#define EXTRACTORS_DEFAULT_HPP

/// COMPONENT
#include "extractor_manager.h"

/// SYSTEM
#include <opencv2/nonfree.hpp>

using namespace vision_evaluator;

/// COMBINATIONS

struct Orb : public ExtractorManager::FeatureDetectorConstructor, public ExtractorManager::DescriptorConstructor {
    EXTRACTOR_IMPLEMENTATION

    void keypoint(Extractor* e, bool complete) {
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

    void descriptor(Extractor* e) {
        Config config = Config::getGlobal();

        e->is_binary = true;
        e->descriptor = config.getDescriptorType();
        e->descriptor_extractor = new cv::ORB();
    }
};
REGISTER_FEATURE_DETECTOR(Orb, ORB);


struct Brisk : public ExtractorManager::FeatureDetectorConstructor, public ExtractorManager::DescriptorConstructor {
    EXTRACTOR_IMPLEMENTATION

    void keypoint(Extractor* e, bool complete) {
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

    void descriptor(Extractor* e) {
        Config config = Config::getGlobal();

        e->is_binary = true;
        e->descriptor = config.getDescriptorType();
        e->descriptor_extractor = new cv::BRISK(config.extractor_threshold, config.octaves);
    }

};
REGISTER_FEATURE_DETECTOR(Brisk, BRISK);


struct Sift : public ExtractorManager::FeatureDetectorConstructor, public ExtractorManager::DescriptorConstructor {
    EXTRACTOR_IMPLEMENTATION

    void keypoint(Extractor* e, bool complete) {
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

    void descriptor(Extractor* e) {
        Config config = Config::getGlobal();

        e->is_binary = false;
        e->descriptor = config.getDescriptorType();
        e->descriptor_extractor = new cv::SiftDescriptorExtractor();
    }

};
REGISTER_FEATURE_DETECTOR(Sift, SIFT);



struct Surf : public ExtractorManager::FeatureDetectorConstructor, public ExtractorManager::DescriptorConstructor {
    EXTRACTOR_IMPLEMENTATION

    void keypoint(Extractor* e, bool complete) {
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

    void descriptor(Extractor* e) {
        Config config = Config::getGlobal();

        e->is_binary = false;
        e->descriptor = config.getDescriptorType();
        e->descriptor_extractor = new cv::SurfDescriptorExtractor();
    }

};
REGISTER_FEATURE_DETECTOR(Surf, SURF);




/// KEYPOINTS ONLY


struct Fast : public ExtractorManager::FeatureDetectorConstructor {
    EXTRACTOR_IMPLEMENTATION

    void keypoint(Extractor* e, bool complete) {
        Config config = Config::getGlobal();

        e->keypoint = config.getKeypointType();
        e->has_orientation = false;
        e->detector = new cv::FastFeatureDetector(config.extractor_threshold, true);
    }
};
REGISTER_FEATURE_DETECTOR(Fast, FAST);



struct Mser : public ExtractorManager::FeatureDetectorConstructor {
    EXTRACTOR_IMPLEMENTATION

    void keypoint(Extractor* e, bool complete) {
        Config config = Config::getGlobal();

        e->keypoint = config.getKeypointType();
        e->has_orientation = true;
        e->detector = new cv::MSER();
    }
};
REGISTER_FEATURE_DETECTOR(Mser, MSER);



struct Star : public ExtractorManager::FeatureDetectorConstructor {
    EXTRACTOR_IMPLEMENTATION

    void keypoint(Extractor* e, bool complete) {
        Config config = Config::getGlobal();

        e->keypoint = config.getKeypointType();
        e->has_orientation = true;
        e->detector = new cv::StarFeatureDetector(16, config.extractor_threshold);
    }
};
REGISTER_FEATURE_DETECTOR(Star, STAR);


struct Gftt : public ExtractorManager::FeatureDetectorConstructor {
    EXTRACTOR_IMPLEMENTATION

    void keypoint(Extractor* e, bool complete) {
        Config config = Config::getGlobal();

        e->keypoint = config.getKeypointType();
        e->has_orientation = true;
        e->detector = new cv::GoodFeaturesToTrackDetector(config.extractor_threshold * 50.0, 0.005);
    }
};
REGISTER_FEATURE_DETECTOR(Gftt, GFTT);


struct GfttHarris : public ExtractorManager::FeatureDetectorConstructor {
    EXTRACTOR_IMPLEMENTATION

    void keypoint(Extractor* e, bool complete) {
        Config config = Config::getGlobal();

        e->keypoint = config.getKeypointType();
        e->has_orientation = true;
        e->detector = new cv::GoodFeaturesToTrackDetector(config.extractor_threshold * 50.0, 0.005, 2, 3, true);
    }
};
REGISTER_FEATURE_DETECTOR(GfttHarris, GFTT_HARRIS);



/// DESCRIPTORS ONLY

struct Brief : public ExtractorManager::DescriptorConstructor {
    EXTRACTOR_IMPLEMENTATION

    void descriptor(Extractor* e) {
        Config config = Config::getGlobal();

        e->is_binary = true;
        e->descriptor = config.getDescriptorType();
        e->descriptor_extractor = new cv::BriefDescriptorExtractor(64);
    }
};
REGISTER_FEATURE_DETECTOR(Brief, BRIEF);


struct Freak : public ExtractorManager::DescriptorConstructor {
    EXTRACTOR_IMPLEMENTATION

    void descriptor(Extractor* e) {
        Config config = Config::getGlobal();

        e->is_binary = true;
        e->descriptor = config.getDescriptorType();
        e->descriptor_extractor = new cv::FREAK();
    }
};
REGISTER_FEATURE_DETECTOR(Freak, FREAK);

#endif // EXTRACTORS_DEFAULT_HPP
