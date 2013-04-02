/// PROJECT
#include <ros/ros_config.h>
#include <analyzer/dumper.h>
#include <ros/trainer_adapter_ros.h>
#include <db_strategy/factory.h>
#include <viz/analyzer_window.h>

/// SYSTEM
#include <boost/program_options.hpp>

namespace po = boost::program_options;

bool feature = false;
bool vj = false;
bool reference = false;

const char* VJONES("viola-jones");
const char* FEATURES("features");
const char* REFERENCES("references");
const char* HELP("help");

bool nothing_provided()
{
    return  !feature && !vj && !reference;
}

int read_parameters(int argc, char** argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()
    (HELP, "show help message")
    (FEATURES, "dump every frame that is detected")
    (REFERENCES, "dump one image for each orientation")
    (VJONES, "dump viola jones features (square images)")
    ;

    po::variables_map vm;
    try {
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);

    } catch(po::unknown_option& e) {
        std::cerr << "Error parsing parameters: " << e.what() << "\n";
        std::cerr << desc << std::endl;
        return 2;
    }

    po::notify(vm);

    feature = vm.count(FEATURES);
    vj = vm.count(REFERENCES);
    reference = vm.count(VJONES);

    if(nothing_provided() || vm.count(HELP)) {
        std::cout << desc << std::endl;
        return 1;
    }

    return 0;
}

int main(int argc, char** argv)
{
    int result = read_parameters(argc, argv);
    if(result != 0) {
        return result;
    }

    ros::init(argc,argv, "feature_dumper");
    ros::NodeHandle nh("~");

    Config cfg = RosConfig::importFromNodeHandle(nh);
    cfg.name = "Feature Dumper";
    cfg.replaceGlobal();

    Dumper dumper;

    nh.param("dump_reference", dumper.dump_reference_images, reference);
    nh.param("dump_features", dumper.dump_feature_data, feature);
    nh.param("dump_viola_jones", dumper.dump_vj_data, vj);

    TrainerAdapterRos node(dumper);
    return node.run<AnalyzerWindow>(argc, argv);
}
