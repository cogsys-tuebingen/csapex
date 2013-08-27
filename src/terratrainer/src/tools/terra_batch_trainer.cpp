#include "terra_batch_trainer.h"
#include <iostream>
#include <fstream>

TerraBatchTrainer::TerraBatchTrainer(const std::string &path) :
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

}

void TerraBatchTrainer::read(std::ifstream &in)
{
    try {
        YAML::Parser parser(in);
        YAML::Node   document;
        parser.GetNextDocument(document);

        /// EXTRACTOR PARAMS
        cv_extraction::ParamsORB  orb;
        if(orb.read(document))
            extracto_params_.reset(new cv_extraction::ParamsORB(orb));
        cv_extraction::ParamsSURF surf;
        if(surf.read(document))
            extracto_params_.reset(new cv_extraction::ParamsSURF(surf));
        cv_extraction::ParamsSIFT sift;
        if(sift.read(document))
            extracto_params_.reset(new cv_extraction::ParamsSIFT(sift));
        cv_extraction::ParamsBRISK brisk;
        if(brisk.read(document))
            extracto_params_.reset(new cv_extraction::ParamsBRISK(brisk));
        cv_extraction::ParamsBRIEF brief;
        if(brief.read(document))
            extracto_params_.reset(new cv_extraction::ParamsBRIEF(brief));
        cv_extraction::ParamsFREAK freak;
        if(freak.read(document))
            extracto_params_.reset(new cv_extraction::ParamsFREAK(freak));
        cv_extraction::ParamsLBP lbp;
        if(lbp.read(document))
            extracto_params_.reset(new cv_extraction::ParamsLBP(lbp));
        cv_extraction::ParamsLTP ltp;
        if(ltp.read(document))
            extracto_params_.reset(new cv_extraction::ParamsLTP(ltp));

        /// KEYPOINT PARAMS
        keypoint_params_.read(document);

        /// PATHS TO THE FILES
        const YAML::Node &paths = document["ROI_FILES"];
        readSequence<std::string>(paths.begin(), paths.end() , buf_roi_paths_);

        /// BUFFER CLASSES
        const YAML::Node &classes = document["CLASSES"];
        readSequence<int>(classes.begin(), classes.end(), buf_classes_);

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
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "CLASSES" << YAML::Value;
    writeSequence<int>(emitter, buf_classes_);
    emitter << YAML::Key << "CLASSES_PALLETTE" << YAML::Value;
    writeSequence<int>(emitter, buf_colors_);
    emitter << YAML::Key << "CLASSIFIER" << YAML::Value;
    emitter << YAML::BeginMap;
    keypoint_params_.write(emitter);
    extracto_params_->write(emitter);
    emitter << YAML::Key << "DATA" << YAML::Value;
    emitter << "" ; //// TREE
    emitter << YAML::EndMap;
}

int main(int argc, char *argv[])
{

    return 0;
}
