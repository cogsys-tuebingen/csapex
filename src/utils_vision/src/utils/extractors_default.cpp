#ifndef EXTRACTORS_DEFAULT_HPP
#define EXTRACTORS_DEFAULT_HPP

/// COMPONENT
#include "extractor_manager.h"

/// SYSTEM
#include <opencv2/nonfree/nonfree.hpp>

using namespace csapex;
using namespace vision;

/// COMBINATIONS

struct Orb : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    struct KeyParams : public ExtractorManager::Params {
        KeyParams() {
            params.push_back(Parameter::declare("extractor_threshold", 0, 200, 50, 1));
            params.push_back(Parameter::declare("levels", 1, 20, 8, 1));
            params.push_back(Parameter::declare("edgeThreshold", 0, 64, 0, 1));
            params.push_back(Parameter::declare("first_level", 0, 8, 0, 1));
            params.push_back(Parameter::declare("WTA_K", 0, 8, 2, 1));
            params.push_back(Parameter::declare("patch_size", 0, 128, 31, 1));

            params.push_back(Parameter::declare("scale", 0.5, 2.0, 1.2, 0.05));

            params.push_back(Parameter::declare("test", false));
        }
    };
    static KeyParams& params() {
        static KeyParams p;
        return p;
    }
    static std::vector<Parameter> usedParameters() {
        return params().params;
    }

    static void keypoint(Extractor* e, const vision::ParameterProvider& param, bool complete) {
        int et          = params().read<int>   (param, "extractor_threshold");
        double scale    = params().read<double>(param, "scale");
        int levels      = params().read<int>   (param, "levels");
        int edge        = params().read<int>   (param, "edgeThreshold");
        int first_level = params().read<int>   (param, "first_level");
        int WTA_K       = params().read<int>   (param, "WTA_K");
        int patch_size  = params().read<int>   (param, "patch_size");

        e->keypoint = "orb";
        e->has_orientation = true;
        e->detector = new cv::ORB((200-et)*10, scale, levels, edge, first_level, WTA_K, cv::ORB::FAST_SCORE, patch_size);

        if(complete) {
            e->is_binary = true;
            e->descriptor = "orb";
            e->descriptor_extractor = e->detector;
        }
    }

    static void descriptor(Extractor* e, const vision::ParameterProvider& param) {
        e->is_binary = true;
        e->descriptor = "orb";
        e->descriptor_extractor = new cv::ORB();
    }
};
REGISTER_FEATURE_DETECTOR(Orb, ORB);
BOOST_STATIC_ASSERT(DetectorTraits<Orb>::HasKeypoint);
BOOST_STATIC_ASSERT(DetectorTraits<Orb>::HasDescriptor);

struct Brisk : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    struct KeyParams : public ExtractorManager::Params {
        KeyParams() {
            params.push_back(Parameter::declare("extractor_threshold", 0, 1000, 50, 1));
            params.push_back(Parameter::declare("octaves", 0, 10, 4, 1));
            params.push_back(Parameter::declare("pattern_scale", 0.2, 10.0, 2.0, 0.1));
        }
    };
    static KeyParams& params() {
        static KeyParams p;
        return p;
    }
    static std::vector<Parameter> usedParameters() {
        return params().params;
    }

    static void keypoint(Extractor* e, const vision::ParameterProvider& param, bool complete) {
        int et       = params().read<int>   (param, "extractor_threshold");
        int octaves  = params().read<int>   (param, "octaves");
        double scale = params().read<double>(param, "pattern_scale");

        e->keypoint = "brisk";
        e->has_orientation = true;
        e->detector = new cv::BRISK(et, octaves, scale);

        if(complete) {
            e->is_binary = true;
            e->descriptor = "brisk";
            e->descriptor_extractor = e->detector;
        }
    }

    static void descriptor(Extractor* e, const vision::ParameterProvider& param) {
        int et      = params().read<int> (param, "extractor_threshold");
        int octaves = params().read<int> (param, "octaves");

        e->is_binary = true;
        e->descriptor = "brisk";
        e->descriptor_extractor = new cv::BRISK(et, octaves);
    }

};
REGISTER_FEATURE_DETECTOR(Brisk, BRISK);
BOOST_STATIC_ASSERT(DetectorTraits<Brisk>::HasKeypoint);
BOOST_STATIC_ASSERT(DetectorTraits<Brisk>::HasDescriptor);


struct Sift : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    struct KeyParams : public ExtractorManager::Params  {
        KeyParams() {
            params.push_back(Parameter::declare("extractor_threshold", 0, 1000, 50, 1));
            params.push_back(Parameter::declare("nOctaveLayers", 0, 5, 3, 1));
            params.push_back(Parameter::declare("contrastThreshold", 0.0, 1.0, 0.04, 0.005));
            params.push_back(Parameter::declare("edgeThreshold", 0.0, 100.0, 10.0, 1.0));
            params.push_back(Parameter::declare("sigma", 0.0, 5.0, 1.6, 0.01));
        }
    };
    static KeyParams& params() {
        static KeyParams p;
        return p;
    }
    static std::vector<Parameter> usedParameters() {
        return params().params;
    }
    static void keypoint(Extractor* e, const vision::ParameterProvider& param, bool complete) {
        int et                   = params().read<int> (param, "extractor_threshold");
        int nOctaveLayers        = params().read<int> (param, "nOctaveLayers");
        double contrastThreshold = params().read<double> (param, "contrastThreshold");
        double edgeThreshold     = params().read<double> (param, "edgeThreshold");
        double sigma             = params().read<double> (param, "sigma");
        e->keypoint = "sift";
        e->has_orientation = true;
        e->detector = new cv::SiftFeatureDetector(et * 0.002 / 3, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);

        if(complete) {
            e->is_binary = false;
            e->descriptor = "sift";
            e->descriptor_extractor = e->detector;
        }
    }

    static void descriptor(Extractor* e, const vision::ParameterProvider& param) {
        e->is_binary = false;
        e->descriptor = "sift";
        e->descriptor_extractor = new cv::SiftDescriptorExtractor();
    }

};
REGISTER_FEATURE_DETECTOR(Sift, SIFT);
BOOST_STATIC_ASSERT(DetectorTraits<Sift>::HasKeypoint);
BOOST_STATIC_ASSERT(DetectorTraits<Sift>::HasDescriptor);



struct Surf : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    struct KeyParams : public ExtractorManager::Params  {
        KeyParams() {
            params.push_back(Parameter::declare("extractor_threshold", 0, 5000, 1000, 1));
            params.push_back(Parameter::declare("nOctaves", 0, 10, 4, 1));
            params.push_back(Parameter::declare("nOctaveLayers", 0, 10, 2, 1));
            params.push_back(Parameter::declare("extended", true));
            params.push_back(Parameter::declare("upright", false));
        }
    };
    static KeyParams& params() {
        static KeyParams p;
        return p;
    }
    static std::vector<Parameter> usedParameters() {
        return params().params;
    }

    static void keypoint(Extractor* e, const vision::ParameterProvider& param, bool complete) {
        int et            = params().read<int>  (param, "extractor_threshold");
        int nOctaves      = params().read<int>  (param, "nOctaves");
        int nOctaveLayers = params().read<int>  (param, "nOctaveLayers");
        bool extended     = params().read<bool> (param, "extended");
        bool upright      = params().read<bool> (param, "upright");

        e->keypoint = "surf";
        e->has_orientation = true;
        e->detector = new cv::SurfFeatureDetector(et, nOctaves, nOctaveLayers, extended, upright);

        if(complete) {
            e->is_binary = false;
            e->descriptor = "surf";
            e->descriptor_extractor = e->detector;
        }
    }

    static void descriptor(Extractor* e, const vision::ParameterProvider& param) {
        e->is_binary = false;
        e->descriptor = "surf";
        e->descriptor_extractor = new cv::SurfDescriptorExtractor();
    }

};
REGISTER_FEATURE_DETECTOR(Surf, SURF);
BOOST_STATIC_ASSERT(DetectorTraits<Surf>::HasKeypoint);
BOOST_STATIC_ASSERT(DetectorTraits<Surf>::HasDescriptor);




/// KEYPOINTS ONLY


struct Fast : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    struct KeyParams : public ExtractorManager::Params  {
        KeyParams() {
            params.push_back(Parameter::declare("extractor_threshold", 0, 200, 50, 1));
            params.push_back(Parameter::declare("nonmaxSuppression", true));
        }
    };
    static KeyParams& params() {
        static KeyParams p;
        return p;
    }
    static std::vector<Parameter> usedParameters() {
        return params().params;
    }

    static void keypoint(Extractor* e, const vision::ParameterProvider& param, bool complete) {
        int et                 = params().read<int>  (param, "extractor_threshold");
        bool nonmaxSuppression = params().read<bool> (param, "nonmaxSuppression");
        e->keypoint = "fast";
        e->has_orientation = false;
        e->detector = new cv::FastFeatureDetector(et, nonmaxSuppression);
    }
};
REGISTER_FEATURE_DETECTOR(Fast, FAST);
BOOST_STATIC_ASSERT(DetectorTraits<Fast>::HasKeypoint);
BOOST_STATIC_ASSERT(!DetectorTraits<Fast>::HasDescriptor);



struct Mser : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    struct KeyParams : public ExtractorManager::Params  {
        KeyParams() {
            params.push_back(Parameter::declare("delta", 0, 50, 5, 1));
            params.push_back(Parameter::declare("minArea", 0, 1000, 60, 1));
            params.push_back(Parameter::declare("maxArea", 0, 30000, 14400, 1));
            params.push_back(Parameter::declare("maxVariation", 0.0, 1.0, 0.25, 0.01));
            params.push_back(Parameter::declare("minDiversity", 0.0, 1.0, 0.2, 0.01));
            params.push_back(Parameter::declare("maxEvolution", 0, 1000, 200, 1));
            params.push_back(Parameter::declare("areaThreshold", 0.0, 2.0, 1.01, 0.01));
            params.push_back(Parameter::declare("minMargin", 0.0, 0.1, 0.003, 0.001));
            params.push_back(Parameter::declare("edgeBlurSize", 0, 50, 5, 1));
        }
    };
    static KeyParams& params() {
        static KeyParams p;
        return p;
    }
    static std::vector<Parameter> usedParameters() {
        return params().params;
    }

    static void keypoint(Extractor* e, const vision::ParameterProvider& param, bool complete) {
        int delta            = params().read<int>    (param, "delta");
        int minArea          = params().read<int>    (param, "minArea");
        int maxArea          = params().read<int>    (param, "maxArea");
        double maxVariation  = params().read<double> (param, "maxVariation");
        double minDiversity  = params().read<double> (param, "minDiversity");
        int maxEvolution     = params().read<int>    (param, "maxEvolution");
        double areaThreshold = params().read<double> (param, "areaThreshold");
        double minMargin     = params().read<double> (param, "minMargin");
        int edgeBlurSize     = params().read<int>    (param, "edgeBlurSize");

        e->keypoint = "mser";
        e->has_orientation = true;
        e->detector = new cv::MSER(delta, minArea, maxArea, maxVariation, minDiversity, maxEvolution, areaThreshold, minMargin, edgeBlurSize);
    }
};
REGISTER_FEATURE_DETECTOR(Mser, MSER);
BOOST_STATIC_ASSERT(DetectorTraits<Mser>::HasKeypoint);
BOOST_STATIC_ASSERT(!DetectorTraits<Mser>::HasDescriptor);



struct Star : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    struct KeyParams : public ExtractorManager::Params  {
        KeyParams() {
            params.push_back(Parameter::declare("extractor_threshold", 0, 1000, 50, 1));
        }
    };
    static KeyParams& params() {
        static KeyParams p;
        return p;
    }
    static std::vector<Parameter> usedParameters() {
        return params().params;
    }

    static void keypoint(Extractor* e, const vision::ParameterProvider& param, bool complete) {
        int et          = params().read<int>   (param, "extractor_threshold");

        e->keypoint = "star";
        e->has_orientation = true;
        e->detector = new cv::StarFeatureDetector(16, et);
    }
};
REGISTER_FEATURE_DETECTOR(Star, STAR);
BOOST_STATIC_ASSERT(DetectorTraits<Star>::HasKeypoint);
BOOST_STATIC_ASSERT(!DetectorTraits<Star>::HasDescriptor);


struct Gftt : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    struct KeyParams : public ExtractorManager::Params  {
        KeyParams() {
            params.push_back(Parameter::declare("extractor_threshold", 0, 1000, 50, 1));
        }
    };
    static KeyParams& params() {
        static KeyParams p;
        return p;
    }
    static std::vector<Parameter> usedParameters() {
        return params().params;
    }

    static void keypoint(Extractor* e, const vision::ParameterProvider& param, bool complete) {
        int et          = params().read<int>   (param, "extractor_threshold");

        e->keypoint = "gftt";
        e->has_orientation = true;
        e->detector = new cv::GoodFeaturesToTrackDetector(et * 50.0, 0.005);
    }
};
REGISTER_FEATURE_DETECTOR(Gftt, GFTT);
BOOST_STATIC_ASSERT(DetectorTraits<Gftt>::HasKeypoint);
BOOST_STATIC_ASSERT(!DetectorTraits<Gftt>::HasDescriptor);


struct GfttHarris : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    struct KeyParams : public ExtractorManager::Params  {
        KeyParams() {
            params.push_back(Parameter::declare("extractor_threshold", 0, 1000, 50, 1));
        }
    };
    static KeyParams& params() {
        static KeyParams p;
        return p;
    }
    static std::vector<Parameter> usedParameters() {
        return params().params;
    }

    static void keypoint(Extractor* e, const vision::ParameterProvider& param, bool complete) {
        int et          = params().read<int>   (param, "extractor_threshold");

        e->keypoint = "gftt_harris";
        e->has_orientation = true;
        e->detector = new cv::GoodFeaturesToTrackDetector(et * 50.0, 0.005, 2, 3, true);
    }
};
REGISTER_FEATURE_DETECTOR(GfttHarris, GFTT_HARRIS);
BOOST_STATIC_ASSERT(DetectorTraits<GfttHarris>::HasKeypoint);
BOOST_STATIC_ASSERT(!DetectorTraits<GfttHarris>::HasDescriptor);



/// DESCRIPTORS ONLY

struct Brief : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    struct KeyParams : public ExtractorManager::Params  {
        KeyParams() {
        }
    };
    static KeyParams& params() {
        static KeyParams p;
        return p;
    }
    static std::vector<Parameter> usedParameters() {
        return params().params;
    }

    static void descriptor(Extractor* e, const vision::ParameterProvider& param) {
        e->is_binary = true;
        e->descriptor = "brief";
        e->descriptor_extractor = new cv::BriefDescriptorExtractor(64);
    }
};
REGISTER_FEATURE_DETECTOR(Brief, BRIEF);
BOOST_STATIC_ASSERT(!DetectorTraits<Brief>::HasKeypoint);
BOOST_STATIC_ASSERT(DetectorTraits<Brief>::HasDescriptor);


struct Freak : public ExtractorManager::ExtractorInitializer {
    EXTRACTOR_IMPLEMENTATION

    struct KeyParams : public ExtractorManager::Params  {
        KeyParams() {
        }
    };
    static KeyParams& params() {
        static KeyParams p;
        return p;
    }
    static std::vector<Parameter> usedParameters() {
        return params().params;
    }

    static void descriptor(Extractor* e, const vision::ParameterProvider& param) {
        e->is_binary = true;
        e->descriptor = "freak";
        e->descriptor_extractor = new cv::FREAK();
    }
};
REGISTER_FEATURE_DETECTOR(Freak, FREAK);
BOOST_STATIC_ASSERT(!DetectorTraits<Freak>::HasKeypoint);
BOOST_STATIC_ASSERT(DetectorTraits<Freak>::HasDescriptor);

#endif // EXTRACTORS_DEFAULT_HPP
