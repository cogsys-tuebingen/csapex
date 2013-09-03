/// PROJECT
#include <adapter/trainer_adapter_static.h>
#include <analyzer/trainer.h>
#include <data/directory_io.h>
#include <db_strategy/factory.h>
#include <config/config.h>
#include <utils/extractor.h>
#include <utils/match_scorer_factory.h>
#include <viz/analyzer_window.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <fstream>
#include <utils/LibUtil/Stopwatch.h>
#include <yaml-cpp/yaml.h>

namespace bfs = boost::filesystem;

int bin_count;
MatchScorer* scorer;

template <typename T>
void readInto(YAML::Node& doc, const std::string& name, T& out_and_default_value)
{
    if(doc.FindValue(name)) {
        doc[name] >> out_and_default_value;
    }
}

template <typename T>
T read(YAML::Node& doc, const std::string& name, T default_value)
{
    T result = default_value;

    if(doc.FindValue(name)) {
        doc[name] >> result;
    }

    return result;
}

bool readBool(YAML::Node& doc, const std::string& name, bool default_value)
{
    bool result = default_value;

    if(doc.FindValue(name)) {
        int tmp = 0;
        doc[name] >> tmp;
        result = tmp != 0;
    }

    return result;
}

void readKeypointDescriptor(YAML::Node& doc, Config& cfg)
{
    std::string descriptor, keypoint;

    doc["keypoint_type"] >> keypoint;
    cfg.setKeypointType(keypoint);

    if(doc.FindValue("descriptor_type")) {
        doc["descriptor_type"] >> descriptor;
        cfg.setDescriptorType(descriptor);
    } else {
        ERROR("no descriptor type");
        throw Extractor::IllegalDescriptorException();
    }

    cfg.setDescriptorType(descriptor);
}

Config read_parameters(int argc, char** argv)
{
    Config cfg = Config::instance();

    DirectoryIO::MAX_IMPORT_PER_DIR = 1500;
    bin_count = 32;

    if(argc > 1) {
        std::stringstream ss;
        for(int i = 1; i < argc; ++i) {
            ss << argv[i] << " ";
        }

        YAML::Parser parser(ss);
        YAML::Node doc;

        if(!parser.GetNextDocument(doc)) {
            ERROR("cannot parse " << ss.str());
            throw;
        } else {
            INFO("parsed: " << ss.str());
        }


        readKeypointDescriptor(doc, cfg);

        double matcher_threshold = cfg("matcher_threshold");
        readInto(doc, "matcher_threshold", matcher_threshold);
        cfg["matcher_threshold"] = matcher_threshold;

        int extractor_threshold = cfg("extractor_threshold");
        readInto(doc, "extractor_threshold", extractor_threshold);
        cfg["extractor_threshold"] = extractor_threshold;

        int min_points = cfg("min_points");
        readInto(doc, "min_points", min_points);
        cfg["min_points"] = min_points;

        int octaves = cfg("octaves");
        readInto(doc, "octaves", octaves);
        cfg["octaves"] = octaves;

        readInto(doc, "bin_count", bin_count);

        readInto(doc, "max_import_per_dir", DirectoryIO::MAX_IMPORT_PER_DIR);

        DirectoryIO::MAX_IMPORT_PER_DIR = std::max(0, DirectoryIO::MAX_IMPORT_PER_DIR);

        bool use_pruning = true;
        cfg["use_pruning"] = readBool(doc, "use_pruning", use_pruning);

        bool crop_test = false;
        cfg["crop_test"]= readBool(doc, "crop_test", crop_test);

        bool interactive = true;
        cfg["interactive"] = readBool(doc, "interactive", interactive);


        std::string scorer_type = read(doc, "scorer", std::string("homography"));
        std::string db_type = read(doc, "db_type", std::string("bin"));

        if(scorer_type == "homography") {
            MatchScorerFactory::setType(MatchScorerFactory::Scorer::HOMOGRAPHY);
        } else if(scorer_type == "clustering") {
            MatchScorerFactory::setType(MatchScorerFactory::Scorer::CLUSTERING);
        } else if(scorer_type == "reprojection") {
            MatchScorerFactory::setType(MatchScorerFactory::Scorer::REPROJECTION);
        } else {
            ERROR("unknown match scorer: " << scorer_type);
            throw;
        }

        cfg["db_type"] = static_cast<int> (Types::Strategy::read(db_type));


        //'{descriptor_type: BRISK, keypoint_type: BRISK, extractor_threshold: 40, min_points: 3, octaves: 4, max_import_per_dir: 150, bin_count: 16}'
    }

    return cfg;
}

int main(int argc, char** argv)
{
    // measurement for overall runtime
    Stopwatch sw;

    // parse arguments
    Config cfg = read_parameters(argc, argv);

    // TODO: don't assert that /Config/RobotDetection is used -> move to Config  /****
    std::string name = cfg.getDescription();
    std::string result_path = cfg("result_dir");
    std::string db_file = result_path + name + ".db";
    std::string db_imgs = result_path + name + "_imgs/";
    std::string result_file = result_path + name + ".txt";
    if(bfs::exists(db_file)) {
        bfs::remove(db_file);
    }
    if(bfs::exists(db_imgs)) {
        bfs::remove_all(db_imgs);
    }
    bfs::create_directories(db_imgs);

    cfg["db_file"] = db_file;
    cfg["db_imgs"] = db_imgs;
    /****/

    cfg["gui_enabled"] = false;
    cfg["name"] = "Batch Trainer";
    cfg.replaceInstance();

    // prepare result output
    std::ofstream out(result_file.c_str());
    INFO("writing to file " << result_file);

    // create the trainer
    Trainer trainer;
    TrainerAdapterStatic node(trainer, out);

    //cfg.replaceGlobal();

    // run the evaluation
    int r = node.run<AnalyzerWindow>(argc, argv);

    // finish up the overall time
    out << "training_time=" << sw.sElapsedDouble() << "s" << std::flush;

    return r;
}
