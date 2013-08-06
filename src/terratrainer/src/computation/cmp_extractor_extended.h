#ifndef CMP_EXTRACTOR_EXTENDED_H
#define CMP_EXTRACTOR_EXTENDED_H
#include "cmp_extractor.h"
#include <yaml-cpp/yaml.h>

class CMPExtractorExt : public CMPExtractor
{
public:
    typedef CMPExtractor::ROI ROI;
    typedef boost::shared_ptr<CMPExtractorExt> Ptr;

    CMPExtractorExt();
    void extractToYAML(YAML::Emitter &emitter, const cv::Mat &img, std::vector<ROI> &rois);

private:
    void      writeMatrix(const cv::Mat &mat, YAML::Emitter &emitter);

};

#endif // CMP_EXTRACTOR_EXTENDED_H
