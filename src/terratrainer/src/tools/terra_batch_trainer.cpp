#include "terra_batch_trainer.h"
#include <iostream>
#include <fstream>
#include <sstream>

TerraBatchTrainer::TerraBatchTrainer(const std::string &path) :
    EXTRACTION_PATH("extraction.yaml"),
    CLASSIFIER_PATH("classifier.yaml"),
    FOREST_PATH("forest.yaml"),
    roi_lib_path_(path)
{
}

void TerraBatchTrainer::run()
{
    std::ifstream in(roi_lib_path_.c_str());
    if(!in.is_open()) {
        std::cerr << "Couldn't open file !" << std::endl;
        return;
    }
    read(in);
    in.close();
    extract();
    train();

    std::ofstream out(CLASSIFIER_PATH.c_str());
    if(!out.is_open()) {
        std::cerr << "Couldn't write file !" << std::endl;
        return;
    }

    write(out);
    out.close();

}

void TerraBatchTrainer::read(std::ifstream &in)
{
    try {
        YAML::Parser parser(in);
        YAML::Node   document;
        parser.GetNextDocument(document);

        /// EXTRACTOR PARAMS
        pattern_extractor_.reset(new CMPPatternExtractorExt);
        feature_extractor_.reset(new CMPFeatureExtractorExt);

        cv_extraction::ParamsORB  orb;
        if(orb.read(document)) {
            pattern_extractor_.reset();
            feature_extractor_->setParams(orb);
            extracto_params_.reset(new cv_extraction::ParamsORB(orb));
        }
        cv_extraction::ParamsSURF surf;
        if(surf.read(document)) {
            pattern_extractor_.reset();
            feature_extractor_->setParams(surf);
            extracto_params_.reset(new cv_extraction::ParamsSURF(surf));
        }
        cv_extraction::ParamsSIFT sift;
        if(sift.read(document)) {
            pattern_extractor_.reset();
            feature_extractor_->setParams(sift);
            extracto_params_.reset(new cv_extraction::ParamsSIFT(sift));
        }
        cv_extraction::ParamsBRISK brisk;
        if(brisk.read(document)) {
            pattern_extractor_.reset();
            feature_extractor_->setParams(brisk);
            extracto_params_.reset(new cv_extraction::ParamsBRISK(brisk));
        }
        cv_extraction::ParamsBRIEF brief;
        if(brief.read(document)) {
            pattern_extractor_.reset();
            feature_extractor_->setParams(brief);
            extracto_params_.reset(new cv_extraction::ParamsBRIEF(brief));
        }
        cv_extraction::ParamsFREAK freak;
        if(freak.read(document)) {
            pattern_extractor_.reset();
            feature_extractor_->setParams(freak);
            extracto_params_.reset(new cv_extraction::ParamsFREAK(freak));
        }
        cv_extraction::ParamsLBP lbp;
        if(lbp.read(document)) {
            feature_extractor_.reset();
            pattern_extractor_->setParams(lbp);
            extracto_params_.reset(new cv_extraction::ParamsLBP(lbp));
        }
        cv_extraction::ParamsLTP ltp;
        if(ltp.read(document)) {
            feature_extractor_.reset();
            pattern_extractor_->setParams(ltp);
            extracto_params_.reset(new cv_extraction::ParamsLTP(ltp));
        }

        /// KEYPOINT PARAMS
        keypoint_params_.read(document);
        forest_params_.read(document);
        feature_extractor_->setKeyPointParams(keypoint_params_);

        /// PATHS TO THE FILES
        const YAML::Node &paths = document["ROI_FILES"];
        readSequence<std::string>(paths.begin(), paths.end() , buf_roi_paths_);

        /// BUFFER CLASSES
        const YAML::Node &classes = document["CLASSES"];
        readClasses(classes.begin(), classes.end());

        /// BUFFER COLORS
        const YAML::Node &colors = document["CLASSES_PALETTE"];
        readSequence<int>(colors.begin(), colors.end(), buf_colors_);

        std::cout << "Loaded settings and roi file pathes!" << std::endl;
    } catch (YAML::Exception e) {
        std::cerr << "Problems reading preferences : '" << e.what() <<"' !" << std::endl;
    }
}

void TerraBatchTrainer::write(std::ofstream &out)
{
    std::ifstream in(FOREST_PATH.c_str());
    if(!in.is_open()) {
        std::cerr << "Resulting forest couldn't be opened!" << std::endl;
        return;
    }

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "CLASSES" << YAML::Value;
    writeClasses(emitter);
    emitter << YAML::Key << "CLASSES_PALETTE" << YAML::Value;
    writeSequence<int>(emitter, buf_colors_);
    emitter << YAML::Key << "CLASSIFIER" << YAML::Value;
    emitter << YAML::BeginMap;
    keypoint_params_.write(emitter);
    extracto_params_->write(emitter);
    emitter << YAML::Key << "DATA" << YAML::Value;
    std::stringstream buf;
    buf << in.rdbuf();
    emitter << buf.str();
    emitter << YAML::EndMap;
    out << emitter.c_str();
    in.close();
}

void TerraBatchTrainer::extract()
{
    std::ofstream out(EXTRACTION_PATH.c_str());
    if(!out.is_open()) {
        std::cerr << "Couldn't not write extraction file!" << std::endl;
        return;
    }

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "data" << YAML::Value;
    emitter << YAML::BeginSeq;
    for(VecStr::iterator it = buf_roi_paths_.begin() ; it != buf_roi_paths_.end() ; it++) {
        extractROIS(*it, emitter);
    }
    emitter << YAML::EndSeq;
    emitter << YAML::EndMap;
    out << emitter.c_str();
    out.close();
}

void TerraBatchTrainer::extractROIS(const std::string &path, YAML::Emitter &emitter)
{
    std::ifstream in(path.c_str());
    if(!in.is_open()) {
        std::cerr << "Couldn't open file '" << path << "' !" << std::endl;
        return;
    }

    std::vector<cv_roi::TerraROI> terra_rois;
    cv::Mat image;

    try {
        YAML::Parser p(in);
        YAML::Node   doc;
        p.GetNextDocument(doc);
        std::string image_path;
        doc["IMAGE"] >> image_path;
        image = cv::imread(image_path);

        const YAML::Node &rois = doc["ROIS"];
        for(YAML::Iterator it = rois.begin() ; it != rois.end() ; it++) {
            cv_roi::TerraROI roi;
            (*it)["class"] >> roi.id.id;
            (*it)["x"]     >> roi.roi.rect.x;
            (*it)["y"]     >> roi.roi.rect.y;
            (*it)["w"]     >> roi.roi.rect.width;
            (*it)["h"]     >> roi.roi.rect.height;
            terra_rois.push_back(roi);
        }

    } catch(YAML::Exception e) {
        std::cerr << "Error reading document '" << e.what() << "' !" << std::endl;
        return;
    }

    if(image.empty()) {
        std::cerr << "Image couldn't be opened!" << std::endl;
        return;
    }

    if(pattern_extractor_ != NULL) {
        pattern_extractor_->extractToYAML(emitter, image, terra_rois);
    }

    if(feature_extractor_ != NULL) {
        feature_extractor_->extractToYAML(emitter, image, terra_rois);
    }

}

void TerraBatchTrainer::train()
{
    CMPRandomForestExt::Ptr random_forest(new CMPRandomForestExt);
    random_forest->setParams(forest_params_);

    bool trained = random_forest->trainFromData(EXTRACTION_PATH);
    if(trained) {
        try {
            random_forest->save(FOREST_PATH);
        } catch (cv::Exception e) {
            std::cerr << "ERROR : " << e.what() << std::endl;
        }

    } else {
        std::cerr << "Couldn't train classifier!" << std::endl;
    }
}

int main(int argc, char *argv[])
{
    if(argc != 2) {
        std::cerr << "Wrong number of arguments - execute 'terraBatchtrainer <file>.yaml" << std::endl;
        return 1;
    }

    TerraBatchTrainer batch(argv[1]);
    batch.run();
    return 0;
}
