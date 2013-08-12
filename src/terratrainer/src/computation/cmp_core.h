#ifndef CMP_CORE_H
#define CMP_CORE_H
/// COMPONENT
#include "params.hpp"
#include "cmp_extractor_extended.h"
#include "cmp_randomforest_extended.h"
#include "extractors.hpp"
#include "roi.hpp"
/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <utils/LibCvTools/grid.hpp>
#include <utils/LibCvTools/terra_decomposition_quadtree.h>
#include <utils/LibCvTools/grid_compute.hpp>


class CMPCore
{
public:
    typedef boost::shared_ptr<CMPCore>                          Ptr;

    CMPCore();
    void        setWorkPath(const std::string &work_path);

    ///         IMAGE
    bool        loadImage(const std::string image_path);
    cv::Mat     getImage() const;
    ///         CLASSIFIER
    bool        load(const std::string path, const std::vector<int> &classRegister);
    std::string forestPath();

    ///     COMPUTATION
    void    compute();
    void    computeGrid();
    void    computeQuadtree();
    bool    hasComputedModel();

    ///     GET VISUALIZABLE RESULTS
    void    getGrid(std::vector<cv_roi::TerraROI> &cells);
    void    getQuad(std::vector<cv_roi::TerraROI> &regions);

    ///     PARAMETERS
    template<class Parameters>
    void setExtractorParameters(Parameters &param)
    {
        cv::DescriptorExtractor* ptr = CMPExtractors::create(param);
        if(param.opp)
            CMPExtractors::makeOpp(ptr);
        extractor_->set(ptr);
        type_ = param.type;
    }
    void    setRandomForestParams(const CMPForestParams &params);
    void    setGridParameters(const CMPGridParams &params);
    void    setQuadParameters(const CMPQuadParams &params);

    ///     TRAINING PREPARATION
    void    setRois(const std::vector<cv_roi::TerraROI> &rois);
    void    addClass(int classID);
    void    removeClass(int classID);
    void    getClasses(std::vector<int> &classes);


private:
    typedef CMPExtractorExt::KeyPoints KeyPoints;

    cv::Mat                                 raw_image_;
    CMPExtractorExt::Ptr                    extractor_;
    CMPExtractorParams::Type                type_;
    CMPRandomForestExt::Ptr                 random_;

    TerraQuadtreeDecomposition::Ptr         quad_decom_;
    CMPQuadParams                           quad_params_;

    boost::shared_ptr<cv_grid::GridTerra>   grid_;
    CMPGridParams                           grid_params_;

    std::string                     work_path_;
    std::string                     file_extraction_;
    std::string                     file_forest_;
    std::vector<cv_roi::TerraROI>   rois_;
    std::vector<int>                classIDs_;


    void extract();
    void train();
};

#endif // CMP_CORE_H
