#include "cmp_core.h"
#include <ostream>
#include <fstream>
#include <set>
#include <time.h>

#include "yaml.hpp"

CMPCore::CMPCore() :
    cv_extractor_(new CMPCVExtractorExt),
    pt_extractor_(new CMPPatternExtractorExt),
    random_(new CMPRandomForestExt),
    grid_(new cv_grid::GridTerra),
    work_path_("/tmp"),
    file_extraction_("/extract.yaml"),
    file_forest_("/forest.yaml")
{
}

void CMPCore::setWorkPath(const std::string &work_path)
{
    work_path_ = work_path;
}

bool CMPCore::loadImage(const std::string image_path)
{
    raw_image_ = cv::imread(image_path);

    return !raw_image_.empty();
}

cv::Mat CMPCore::getImage() const
{
    return raw_image_.clone();
}

void CMPCore::reload()
{
    if(!random_->load(forestPath().c_str())) {
        std::cerr << "Error reading forest file!" << std::endl;
    }
}


std::string CMPCore::forestPath()
{
    return work_path_ + file_forest_;
}

void CMPCore::compute()
{

    /// STEP 1 - THE EXTRACTION
    extract();
    /// STEP 2 - THE TRAINING
    train();
}

void CMPCore::computeGrid()
{
    if(!random_->isTrained()) {
        std::cerr << "No trained random forest! - Therefore not compitung grid!" << std::endl;
        return;
    }

    if(grid_ == NULL)
        grid_.reset(new cv_grid::GridTerra);

    /// PARAMETERS
    cv_grid::AttrTerrainFeature::Params p;
    p.extractor       = cv_extractor_.get();
    p.classifier      = random_.get();
    p.key             = keypoint_params_;
    p.color_extension = ex_params_->color_extension;
    p.large_descriptor= ex_params_->combine_descriptors;
    p.max_octave      = ex_params_->octaves;
    p.use_max_prob    = ex_params_->use_max_prob;

    /// CALCULATE GRID SIZE
    int height = raw_image_.rows / grid_params_.cell_height;
    int width  = raw_image_.cols / grid_params_.cell_width;

    cv_grid::prepare_grid<cv_grid::AttrTerrainFeature>(*grid_, raw_image_, height, width, p, cv::Mat());
}

void CMPCore::computeQuadtree()
{
    if(!random_->isTrained()) {
        std::cerr << "No trained random forest! - Therefore not compitung quad tree decomposition!" << std::endl;
        return;
    }
    cv::Size min_size(quad_params_.min_width, quad_params_.min_height);
    TerraDecomClassifierFeature   *classifier = new TerraDecomClassifierFeature(quad_params_.min_prob,
                                                                                random_.get(),
                                                                                cv_extractor_.get(),
                                                                                keypoint_params_,
                                                                                ex_params_->combine_descriptors,
                                                                                ex_params_->color_extension,
                                                                                ex_params_->octaves);

    TerraQuadtreeDecomposition *decom = new TerraQuadtreeDecomposition(raw_image_,min_size, classifier);
    quad_decom_.reset(decom);

    /// ITERATE
    quad_decom_->auto_iterate();
}

bool CMPCore::hasComputedModel()
{
    return random_->isTrained();
}

void CMPCore::getGrid(std::vector<cv_roi::TerraROI> &cells)
{
    if(grid_ == NULL)
        return;

    cv_grid::GridTerra &terra = *grid_;
    for(int i = 0 ; i < terra.rows() ; i++) {
        for(int j = 0 ; j < terra.cols() ; j++) {
            cv_grid::GridCellTerra &cell = terra(i,j);
            cv_roi::TerraROI tr;
            tr.roi.rect = cell.bounding;
            tr.id.id    = cell.attributes.classID;
            tr.id.prob  = cell.attributes.probability;
            cells.push_back(tr);
        }
    }
}

void CMPCore::getQuad(std::vector<cv_roi::TerraROI> &regions)
{
    if(quad_decom_ != NULL)
        quad_decom_->regions(regions);
}

void CMPCore::setExtractorParameters(cv_extraction::ExtractorParams &params)
{
    switch(params.type) {
    case cv_extraction::ExtractorParams::ORB:
        cv_extractor_->setParams(static_cast<cv_extraction::ParamsORB&>(params));
        ex_params_.reset(new cv_extraction::ParamsORB(static_cast<cv_extraction::ParamsORB&>(params)));
        break;
    case cv_extraction::ExtractorParams::SURF:
        cv_extractor_->setParams(static_cast<cv_extraction::ParamsSURF&>(params));
        ex_params_.reset(new cv_extraction::ParamsSURF(static_cast<cv_extraction::ParamsSURF&>(params)));
        break;
    case cv_extraction::ExtractorParams::SIFT:
        cv_extractor_->setParams(static_cast<cv_extraction::ParamsSIFT&>(params));
        ex_params_.reset(new cv_extraction::ParamsSIFT(static_cast<cv_extraction::ParamsSIFT&>(params)));
        break;
    case cv_extraction::ExtractorParams::BRIEF:
        cv_extractor_->setParams(static_cast<cv_extraction::ParamsBRIEF&>(params));
        ex_params_.reset(new cv_extraction::ParamsBRIEF(static_cast<cv_extraction::ParamsBRIEF&>(params)));
        break;
    case cv_extraction::ExtractorParams::BRISK:
        cv_extractor_->setParams(static_cast<cv_extraction::ParamsBRISK&>(params));
        ex_params_.reset(new cv_extraction::ParamsBRISK(static_cast<cv_extraction::ParamsBRISK&>(params)));
        break;
    case cv_extraction::ExtractorParams::FREAK:
        cv_extractor_->setParams(static_cast<cv_extraction::ParamsFREAK&>(params));
        ex_params_.reset(new cv_extraction::ParamsFREAK(static_cast<cv_extraction::ParamsFREAK&>(params)));
        break;
    case cv_extraction::ExtractorParams::LBP:
        pt_extractor_->setParams(static_cast<cv_extraction::ParamsLBP&>(params));
        ex_params_.reset(new cv_extraction::ParamsLBP(static_cast<cv_extraction::ParamsLBP&>(params)));
        break;
    case cv_extraction::ExtractorParams::LTP:
        pt_extractor_->setParams(static_cast<cv_extraction::ParamsLTP&>(params));
        ex_params_.reset(new cv_extraction::ParamsLTP(static_cast<cv_extraction::ParamsLTP&>(params)));
        break;
    }

    cv_extractor_->setKeyPointParams(keypoint_params_);

}

void CMPCore::setKeyPointParameters(const cv_extraction::KeypointParams &params)
{
    quad_decom_.reset();
    grid_.reset();
    keypoint_params_.angle      = params.angle;
    keypoint_params_.octave     = params.octave;
    keypoint_params_.scale      = params.scale;
    keypoint_params_.soft_crop  = params.soft_crop;
    keypoint_params_.calc_angle = params.calc_angle;
}

void CMPCore::write(YAML::Emitter &emitter) const
{
    keypoint_params_.write(emitter);
    if(ex_params_ != NULL)
        ex_params_->write(emitter);
}

void CMPCore::setRandomForestParams(const CMPForestParams &params)
{
    random_->setParams(params);
}

void CMPCore::setGridParameters(const CMPGridParams &params)
{
    grid_.reset();
    grid_params_ = params;
}

void CMPCore::setQuadParameters(const CMPQuadParams &params)
{
    quad_decom_.reset();
    quad_params_ = params;
}

void CMPCore::setRois(const std::vector<cv_roi::TerraROI> &rois)
{
    rois_ = rois;
}

void CMPCore::addClass(int classID)
{
    classIDs_.push_back(classID);
}

void CMPCore::removeClass(int classID)
{
    std::vector<int>::iterator it = std::find(classIDs_.begin(), classIDs_.end(), classID);
    if(it != classIDs_.end()) {
        classIDs_.erase(it);
    }
}

void CMPCore::getClasses(std::vector<int> &classes)
{
    classes = classIDs_;
}

void CMPCore::extract()
{
    std::ofstream  out((work_path_ + file_extraction_).c_str());
    YAML::Emitter  emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "data" << YAML::Value;
    if(ex_params_->type != cv_extraction::ExtractorParams::LBP &&
            ex_params_->type != cv_extraction::ExtractorParams::LTP)
        cv_extractor_->extractToYAML(emitter, raw_image_, rois_);
    else
        pt_extractor_->extractToYAML(emitter, raw_image_, rois_);

    emitter << YAML::EndMap;
    out << emitter.c_str();
    out.close();
}

void CMPCore::train()
{
    bool trained = random_->trainFromData(work_path_ + file_extraction_);
    if(trained) {
        try {
            random_->save(work_path_ + file_forest_);
        } catch (cv::Exception e) {
            std::cerr << "ERROR : " << e.what() << std::endl;
        }

    } else {
        std::cerr << "Couldn't train classifier!" << std::endl;
    }
}
