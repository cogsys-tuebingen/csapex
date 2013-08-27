#ifndef TERRA_BATCH_TRAINER_H
#define TERRA_BATCH_TRAINER_H
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <vector>
#include <utils/LibCvTools/extractor_params.h>
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
    std::string                             roi_lib_path_;
    std::vector<std::string>                buf_roi_paths_;
    std::vector<int>                        buf_classes_;
    std::vector<int>                        buf_colors_;
    cv_extraction::KeypointParams           keypoint_params_;
    cv_extraction::ExtractorParams::Ptr     extracto_params_;

    void read(std::ifstream &in);
    void write(std::ofstream &out);

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
