#include "cmp_core.h"
#include <ostream>
#include <fstream>
#include <set>
#include <time.h>
#include "extractors.hpp"
#include "yaml.hpp"

CMPCore::CMPCore() :
    extractor_(new CMPExtractorExt),
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
    /// PARAMETERS
    cv_grid::AttrTerrainClass::Params p;
    p.extractor  = extractor_.get();
    p.classifier = random_.get();

    /// CALCULATE GRID SIZE
    int height = raw_image_.rows / grid_params_.cell_height;
    int width  = raw_image_.cols / grid_params_.cell_width;

    cv_grid::prepare_grid<cv_grid::AttrTerrainClass>(*grid_, raw_image_, height, width, p);
}

void CMPCore::computeQuadtree()
{
    cv::Size min_size(quad_params_.min_width, quad_params_.min_height);
    TerraDecomClassifier       *classifier = new TerraDecomClassifier(quad_params_.min_prob, random_.get(), extractor_.get());
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
    extractor_->extractToYAML(emitter, raw_image_, rois_);
    emitter << YAML::EndMap;
    out << emitter.c_str();
    out.close();
}

void CMPCore::train()
{
    bool trained = random_->trainFromData(work_path_ + file_extraction_);
    if(trained) {
        random_->save(work_path_ + file_forest_);
    } else {
        std::cerr << "Couldn't train classifier!" << std::endl;
    }
}
