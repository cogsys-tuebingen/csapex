#include "cmp_core.h"
#include <ostream>
#include <fstream>
#include <set>
#include <time.h>

#include "cmp_extractors.hpp"
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

bool CMPCore::load(const std::string path, const std::vector<int> &classRegister)
{
    classIDs_ = classRegister;
    return random_->load(path.c_str());
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
    cv_grid::AttrTerrainClass::Params p;
    p.extractor  = cv_extractor_.get();
    p.classifier = random_.get();
    p.angle = keypoint_params_.angle;
    p.scale = keypoint_params_.scale;

    /// CALCULATE GRID SIZE
    int height = raw_image_.rows / grid_params_.cell_height;
    int width  = raw_image_.cols / grid_params_.cell_width;

    cv_grid::prepare_grid<cv_grid::AttrTerrainClass>(*grid_, raw_image_, height, width, p, cv::Mat(), 1.0, keypoint_params_.soft_crop);
}

void CMPCore::computeQuadtree()
{
    if(!random_->isTrained()) {
        std::cerr << "No trained random forest! - Therefore not compitung quad tree decomposition!" << std::endl;
        return;
    }
    cv::Size min_size(quad_params_.min_width, quad_params_.min_height);
    TerraDecomClassifier       *classifier = new TerraDecomClassifier(quad_params_.min_prob,
                                                                      random_.get(),
                                                                      cv_extractor_.get(),
                                                                      keypoint_params_.soft_crop,
                                                                      keypoint_params_.scale,
                                                                      keypoint_params_.angle);

    TerraQuadtreeDecomposition *decom = new TerraQuadtreeDecomposition(raw_image_,min_size, classifier);
    quad_decom_.reset(decom);

    /// ITERATE
    decom->auto_iterate();
}

bool CMPCore::hasComputedModel()
{
    return random_->isTrained();
}

void CMPCore::getGrid(std::vector<cv_roi::TerraROI> &cells)
{
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
    quad_decom_->regions(regions);
}

void CMPCore::setExtractorParameters(CMPExtractorParams &params)
{
    switch(params.type) {
    case CMPExtractorParams::ORB:
        cv_extractor_->setParams(static_cast<CMPParamsORB&>(params));
        break;
    case CMPExtractorParams::SURF:
        cv_extractor_->setParams(static_cast<CMPParamsSURF&>(params));
        break;
    case CMPExtractorParams::SIFT:
        cv_extractor_->setParams(static_cast<CMPParamsSIFT&>(params));
        break;
    case CMPExtractorParams::BRIEF:
        cv_extractor_->setParams(static_cast<CMPParamsBRIEF&>(params));
        break;
    case CMPExtractorParams::BRISK:
        cv_extractor_->setParams(static_cast<CMPParamsBRISK&>(params));
        break;
    case CMPExtractorParams::FREAK:
        cv_extractor_->setParams(static_cast<CMPParamsFREAK&>(params));
        break;
    case CMPExtractorParams::LBP:
        pt_extractor_->setParams(static_cast<CMPParamsLBP&>(params));
        break;
    case CMPExtractorParams::LTP:
        pt_extractor_->setParams(static_cast<CMPParamsLTP&>(params));
        break;
    }

    cv_extractor_->setKeyPointParams(keypoint_params_);
    type_ = params.type;

}

void CMPCore::setRandomForestParams(const CMPForestParams &params)
{
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

void CMPCore::setKeyPointParameters(const CMPKeypointParams &params)
{
    keypoint_params_ = params;
}

void CMPCore::setRois(const std::vector<cv_roi::TerraROI> &rois)
{
    rois_ = rois;
}

void CMPCore::saveRois(const std::string path)
{
    std::map<int,int> counts;

    foreach (cv_roi::TerraROI roi, rois_) {
        if(counts.find(roi.id.id) != counts.end())
            counts[roi.id.id]++;
        else
            counts.insert(std::pair<int,int>(roi.id.id, 0));

        std::stringstream roi_path;
        roi_path << path << roi.id.id << "/" << counts[roi.id.id] << ".jpg";
        cv::Mat cv_roi(raw_image_, roi.roi.rect);
        cv::imwrite(roi_path.str(), cv_roi);
    }
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
    if(type_ != CMPExtractorParams::LBP && type_ != CMPExtractorParams::LTP)
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
