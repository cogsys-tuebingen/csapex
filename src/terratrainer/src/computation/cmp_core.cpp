#include "cmp_core.h"
#include <ostream>
#include <fstream>
#include <set>
#include <time.h>

#include "yaml.hpp"
#include "cmp_extraction.hpp"
#include <utils/LibCvTools/terra_decomposition_classifier.h>

CMPCore::CMPCore() :
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

    /// PARAMETERS
    prepareExtractor();

    /// CALCULATE GRID SIZE
    int height = raw_image_.rows / grid_params_.cell_height;
    int width  = raw_image_.cols / grid_params_.cell_width;

    grid_.reset(new cv_grid::GridTerra);
    cv_grid::AttrTerrain::Params p;
    p.extractor       = extractor_;
    p.classifier      = random_;
    p.key             = keypoint_params_;
    p.use_max_prob    = ext_params_->use_max_prob;
    cv_grid::prepare_grid<cv_grid::AttrTerrain>
            (*boost::dynamic_pointer_cast<cv_grid::GridTerra>(grid_), raw_image_, height, width, p, cv::Mat());
}

void CMPCore::computeQuadtree()
{
    prepareExtractor();
    if(!random_->isTrained()) {
        std::cerr << "No trained random forest! - Therefore not compitung quad tree decomposition!" << std::endl;
        return;
    }

        cv::Size min_size(quad_params_.min_width, quad_params_.min_height);
        TerraDecomClassifier::Ptr classifier(new TerraDecomClassifier(quad_params_.min_prob));
        classifier->setClassifier(random_);
        classifier->setExtractor(extractor_);
        classifier->setUseMaxProb(ext_params_->use_max_prob);
        quad_decom_.reset(new TerraQuadtreeDecomposition(raw_image_,min_size, classifier));

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
        ext_params_.reset(new cv_extraction::ParamsORB(static_cast<cv_extraction::ParamsORB&>(params)));
        break;
    case cv_extraction::ExtractorParams::SURF:
        ext_params_.reset(new cv_extraction::ParamsSURF(static_cast<cv_extraction::ParamsSURF&>(params)));
        break;
    case cv_extraction::ExtractorParams::SIFT:
        ext_params_.reset(new cv_extraction::ParamsSIFT(static_cast<cv_extraction::ParamsSIFT&>(params)));
        break;
    case cv_extraction::ExtractorParams::BRIEF:
        ext_params_.reset(new cv_extraction::ParamsBRIEF(static_cast<cv_extraction::ParamsBRIEF&>(params)));
        break;
    case cv_extraction::ExtractorParams::BRISK:
        ext_params_.reset(new cv_extraction::ParamsBRISK(static_cast<cv_extraction::ParamsBRISK&>(params)));
        break;
    case cv_extraction::ExtractorParams::FREAK:
        ext_params_.reset(new cv_extraction::ParamsFREAK(static_cast<cv_extraction::ParamsFREAK&>(params)));
        break;
    case cv_extraction::ExtractorParams::LBP:
        ext_params_.reset(new cv_extraction::ParamsLBP(static_cast<cv_extraction::ParamsLBP&>(params)));
        break;
    case cv_extraction::ExtractorParams::LTP:
        ext_params_.reset(new cv_extraction::ParamsLTP(static_cast<cv_extraction::ParamsLTP&>(params)));
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
    if(ext_params_ != NULL)
        ext_params_->write(emitter);

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
    cv_extraction::FeatureExtractor *feat;
    cv_extraction::PatternExtractor *patt;

    if(ext_params_->type == cv_extraction::ExtractorParams::LBP ||
            ext_params_->type == cv_extraction::ExtractorParams::LTP) {
        patt = new cv_extraction::PatternExtractor;
        extractor_.reset(patt);
    } else {
        feat = new cv_extraction::FeatureExtractor;
        extractor_.reset(feat);
    }


    switch(ext_params_->type) {
    case cv_extraction::ExtractorParams::ORB:
        feat->setParams(*boost::shared_static_cast<cv_extraction::ParamsORB>(ext_params_));
        feat->setKeyPointParams(keypoint_params_);
        break;
    case cv_extraction::ExtractorParams::SURF:
        feat->setParams(*boost::shared_static_cast<cv_extraction::ParamsSURF>(ext_params_));
        feat->setKeyPointParams(keypoint_params_);
        break;
    case cv_extraction::ExtractorParams::SIFT:
        feat->setParams(*boost::shared_static_cast<cv_extraction::ParamsSIFT>(ext_params_));
        feat->setKeyPointParams(keypoint_params_);
        break;
    case cv_extraction::ExtractorParams::BRIEF:
        feat->setParams(*boost::shared_static_cast<cv_extraction::ParamsBRIEF>(ext_params_));
        feat->setKeyPointParams(keypoint_params_);
        break;
    case cv_extraction::ExtractorParams::BRISK:
        feat->setParams(*boost::shared_static_cast<cv_extraction::ParamsBRISK>(ext_params_));
        feat->setKeyPointParams(keypoint_params_);
        break;
    case cv_extraction::ExtractorParams::FREAK:
        feat->setParams(*boost::shared_static_cast<cv_extraction::ParamsFREAK>(ext_params_));
        feat->setKeyPointParams(keypoint_params_);
        break;
    case cv_extraction::ExtractorParams::LBP:
        patt->setParams(*boost::shared_static_cast<cv_extraction::ParamsLBP>(ext_params_));
        break;
    case cv_extraction::ExtractorParams::LTP:
        patt->setParams(*boost::shared_static_cast<cv_extraction::ParamsLTP>(ext_params_));
        break;
    }

}

void CMPCore::extract()
{
    prepareExtractor();

    std::ofstream  out((work_path_ + file_extraction_).c_str());
    YAML::Emitter  emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "data" << YAML::Value;
    emitter << YAML::BeginSeq;
    if(state_ != NULL)
        CMPExtraction::extractToYAML(emitter, raw_image_, extractor_, rois_, state_);
    else
        CMPExtraction::extractToYAML(emitter, raw_image_, extractor_, rois_);
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
