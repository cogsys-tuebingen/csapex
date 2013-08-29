#ifndef TERRA_BATCH_TRAINER_H
#define TERRA_BATCH_TRAINER_H
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <vector>
#include <utils/LibCvTools/extractor_params.h>
#include <computation/cmp_extractor_extended.h>
#include <computation/cmp_randomforest_extended.h>
#include <yaml-cpp/yaml.h>

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

    const std::string                       EXTRACTION_PATH;
    const std::string                       CLASSIFIER_PATH;
    const std::string                       FOREST_PATH;


    std::string                             roi_lib_path_;
    VecStr                                  buf_roi_paths_;
    VecInt                                  buf_classes_;
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


    template<class T>
    void readSequence(const YAML::Iterator &begin, const YAML::Iterator &end,
                      std::vector<T> &buffer)
    {
        for(YAML::Iterator it = begin ; it != end ; it++) {
            T value;
            (*it) >> value;
            buffer.push_back(value);
        }
    }

    void readClasses(const YAML::Iterator &begin, const YAML::Iterator &end)
    {

        for(YAML::Iterator it = begin ; it != end ; it++) {
            int class_id;
            int color;
            std::string info;

            (*it)["id"] >> class_id;
            (*it)["color"] >> color;
            (*it)["info"]  >> info;

            buf_classes_.push_back(class_id);
            buf_classes_colors_.push_back(color);
            buf_classes_infos_.push_back(info);
        }
    }

    void writeClasses(YAML::Emitter &emitter)
    {
        emitter << YAML::BeginSeq;
        for(int i = 0 ; i < buf_classes_.size() ; i++) {
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "id"   << YAML::Value << buf_classes_[i];
            emitter << YAML::Key << "color"<< YAML::Value << buf_classes_colors_[i];
            emitter << YAML::Key << "info" << YAML::Value << buf_classes_infos_[i];
            emitter << YAML::EndMap;
        }
        emitter << YAML::EndSeq;
    }

    template<class T>
    void writeSequence(YAML::Emitter &emitter, const std::vector<T> &buffer)
    {
        emitter << YAML::BeginSeq;
        for(typename std::vector<T>::const_iterator it = buffer.begin() ; it != buffer.end() ; it++)
            emitter << *it;
        emitter << YAML::EndSeq;
    }


    //    Semaphore       available_extractors_;
    
};

#endif // TERRA_BATCH_TRAINER_H
