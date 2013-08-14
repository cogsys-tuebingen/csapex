#ifndef CMP_EXTRACTOR_EXTENDED_H
#define CMP_EXTRACTOR_EXTENDED_H
#include <utils/LibCvTools/extractor.h>
#include <yaml-cpp/yaml.h>
#include <roi.hpp>

class CMPExtractorExt : public Extractor
{
public:
    typedef boost::shared_ptr<CMPExtractorExt> Ptr;

    CMPExtractorExt();
    void extractToYAML(YAML::Emitter &emitter, const cv::Mat &img, std::vector<cv_roi::TerraROI> &rois, const bool soft_crop = true);

private:
    void      writeMatrix(const cv::Mat &mat, YAML::Emitter &emitter);

};

#endif // CMP_EXTRACTOR_EXTENDED_H
