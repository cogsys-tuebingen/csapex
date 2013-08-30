#ifndef TERRA_BATCH_TRAINER_H
#define TERRA_BATCH_TRAINER_H
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <vector>
#include <utils/LibCvTools/extractor_params.h>
#include <computation/cmp_extractor_extended.h>
#include <computation/cmp_randomforest_extended.h>

namespace YAML {
class Emitter;
class Iterator;
}

namespace cv_extraction {
class Extractor;
}

class TerraBatchTrainer
{
public:
    TerraBatchTrainer(const std::string &path);
    void run();

private:
    typedef std::vector<std::string>        VecStr;
    typedef std::vector<int>                VecInt;
    typedef std::vector<uchar>              VecUCh;

    const std::string                       EXTRACTION_PATH;
    const std::string                       CLASSIFIER_PATH;
    const std::string                       FOREST_PATH;


    std::string                             roi_lib_path_;
    VecStr                                  buf_roi_paths_;
    VecUCh                                  buf_classes_;
    VecInt                                  buf_classes_colors_;
    VecStr                                  buf_classes_infos_;
    VecInt                                  buf_colors_;
    cv_extraction::KeypointParams           keypoint_params_;
    cv_extraction::ExtractorParams::Ptr     extracto_params_;
    CMPForestParams                         forest_params_;
    cv_extraction::Extractor::Ptr           extractor_;

    void read(std::ifstream &in);
    void write(std::ofstream &out);
    void extract();
    void extractROIS(const std::string &path, YAML::Emitter &emitter);
    void train();


    void readClasses(const YAML::Iterator &begin, const YAML::Iterator &end);
    void writeClasses(YAML::Emitter &emitter);
    
};

#endif // TERRA_BATCH_TRAINER_H
