#include "cmp_core.h"
#include <ostream>
#include <fstream>
#include <set>
#include <time.h>

#include "yaml.hpp"

CMPCore::CMPCore() :
    cv_extractor_(new CMPFeatureExtractorExt),
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
    prepareExtractor();

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
    prepareExtractor();
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
        ex_params_.reset(new cv_extraction::ParamsORB(static_cast<cv_extraction::ParamsORB&>(params)));
        break;
    case cv_extraction::ExtractorParams::SURF:
        ex_params_.reset(new cv_extraction::ParamsSURF(static_cast<cv_extraction::ParamsSURF&>(params)));
        break;
    case cv_extraction::ExtractorParams::SIFT:
        ex_params_.reset(new cv_extraction::ParamsSIFT(static_cast<cv_extraction::ParamsSIFT&>(params)));
        break;
    case cv_extraction::ExtractorParams::BRIEF:
        ex_params_.reset(new cv_extraction::ParamsBRIEF(static_cast<cv_extraction::ParamsBRIEF&>(params)));
        break;
    case cv_extraction::ExtractorParams::BRISK:
        ex_params_.reset(new cv_extraction::ParamsBRISK(static_cast<cv_extraction::ParamsBRISK&>(params)));
        break;
    case cv_extraction::ExtractorParams::FREAK:
        ex_params_.reset(new cv_extraction::ParamsFREAK(static_cast<cv_extraction::ParamsFREAK&>(params)));
        break;
    case cv_extraction::ExtractorParams::LBP:
        ex_params_.reset(new cv_extraction::ParamsLBP(static_cast<cv_extraction::ParamsLBP&>(params)));
        break;
    case cv_extraction::ExtractorParams::LTP:
        ex_params_.reset(new cv_extraction::ParamsLTP(static_cast<cv_extraction::ParamsLTP&>(params)));
        break;
    }
}

void CMPCore::setKeyPointParameters(const cv_extraction::KeypointParams &params)
{
    keypoint_params_.angle      = params.angle;
    keypoint_params_.octave     = params.octave;
    keypoint_params_.scale      = params.scale;
    keypoint_params_.soft_crop  = params.soft_crop;
    keypoint_params_.calc_angle = params.calc_angle;
}

void CMPCore::write(YAML::Emitter &emitter) const
{
    keypoint_params_.write(emitter);
    forest_params_.write(emitter);
    if(ex_params_ != NULL)
        ex_params_->write(emitter);

}

void CMPCore::setRandomForestParams(const CMPForestParams &params)
{
    forest_params_ = params;
    random_->setParams(params);
}

void CMPCore::setGridParameters(const CMPGridParams &params)
{
    grid_params_ = params;
}

void CMPCore::setQuadParameters(const CMPQuadParams &params)
{
    quad_params_ = params;
}

void CMPCore::setRois(const std::vector<cv_roi::TerraROI> &rois)
{
    rois_ = rois;
}

void CMPCore::setStatePublisher(CMPStatePublisher::Ptr ptr)
{
    state_ = ptr;
}

void CMPCore::unsetStatePublisher()
{
    state_.reset();
}

void CMPCore::prepareExtractor()
{
    switch(ex_params_->type) {
    case cv_extraction::ExtractorParams::ORB:
        cv_extractor_->setParams(*boost::shared_static_cast<cv_extraction::ParamsORB>(ex_params_));
        break;
    case cv_extraction::ExtractorParams::SURF:
        cv_extractor_->setParams(*boost::shared_static_cast<cv_extraction::ParamsSURF>(ex_params_));
        break;
    case cv_extraction::ExtractorParams::SIFT:
        cv_extractor_->setParams(*boost::shared_static_cast<cv_extraction::ParamsSIFT>(ex_params_));
        break;
    case cv_extraction::ExtractorParams::BRIEF:
        cv_extractor_->setParams(*boost::shared_static_cast<cv_extraction::ParamsBRIEF>(ex_params_));
        break;
    case cv_extraction::ExtractorParams::BRISK:
        cv_extractor_->setParams(*boost::shared_static_cast<cv_extraction::ParamsBRISK>(ex_params_));
        break;
    case cv_extraction::ExtractorParams::FREAK:
        cv_extractor_->setParams(*boost::shared_static_cast<cv_extraction::ParamsFREAK>(ex_params_));
        break;
    case cv_extraction::ExtractorParams::LBP:
        pt_extractor_->setParams(*boost::shared_static_cast<cv_extraction::ParamsLBP>(ex_params_));
        break;
    case cv_extraction::ExtractorParams::LTP:
        pt_extractor_->setParams(*boost::shared_static_cast<cv_extraction::ParamsLTP>(ex_params_));
        break;
    }
    cv_extractor_->setKeyPointParams(keypoint_params_);
}

void CMPCore::extract()
{
    prepareExtractor();

    std::ofstream  out((work_path_ + file_extraction_).c_str());
    YAML::Emitter  emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "data" << YAML::Value;
    emitter << YAML::BeginSeq;
    if(ex_params_->type != cv_extraction::ExtractorParams::LBP &&
            ex_params_->type != cv_extraction::ExtractorParams::LTP) {
        if(state_ != NULL)
            cv_extractor_->extractToYAML(emitter, raw_image_, rois_, state_);
        else
            cv_extractor_->extractToYAML(emitter, raw_image_, rois_);
    } else {
        if(state_ != NULL)
            pt_extractor_->extractToYAML(emitter, raw_image_, rois_, state_);
        else
            pt_extractor_->extractToYAML(emitter, raw_image_, rois_);
    }
    emitter << YAML::EndSeq;
    emitter << YAML::EndMap;
    out << emitter.c_str();
    out.close();
}

void CMPCore::train()
{
    state_->publish(std::make_pair(0,0));

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
