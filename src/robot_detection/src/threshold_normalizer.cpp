///// PROJECT
//#include <analyzer/context_static.h>
//#include <adapter/trainer_static.h>
//#include <data/matchable.h>
//#include <data/frame.h>
//#include <data/frame_io.h>
//#include <db_strategy/factory.h>
//#include <robot_detection/GlobalConfig.h>
//#include <tools/extractor.h>
//#include <tools/matcher.h>
//#include <tools/match_scorer_factory.h>

///// SYSTEM
//#include <boost/filesystem.hpp>
//#include <fstream>
//#include <yaml-cpp/yaml.h>
//#include <utils/LibUtil/Stopwatch.h>

//namespace bfs = boost::filesystem;

//int bin_count = 1;

//double low = 100;
//double high = Extractor::TARGET_MAX_FEATURE_COUNT;

//long current_features = 0;
//int current_count = 0;
//std::string batch_dir;

//std::vector<std::string> training, validation, test_positive, test_negative, negative;

//void analyze(std::vector<boost::shared_ptr<Frame> >& vec)
//{
//    for(std::vector<boost::shared_ptr<Frame> >::iterator it = vec.begin(); it != vec.end(); ++it){
//        current_features += (*it)->keypoints.size();
//        current_count++;
//    }

//    vec.clear();
//}

//boost::function<void(std::vector<boost::shared_ptr<Frame> >&)> analyze_fkt = analyze;


//double count_features(const std::string &batch_dir, ContextStatic &context)
//{
//    current_features = 0;
//    current_count = 0;
//    std::vector<std::string>::iterator it;
//    for(it = training.begin(); it != training.end(); ++it){
//        context.import_directory(batch_dir + *it + "/positive", false, &analyze_fkt, true);
//    }
//    for(it = validation.begin(); it != validation.end(); ++it){
//        context.import_directory(batch_dir + *it + "/positive", false, &analyze_fkt, true);
//    }
//    for(it = test_positive.begin(); it != test_positive.end(); ++it){
//        context.import_directory(batch_dir + *it + "/positive", false, &analyze_fkt, true);
//    }

//    for(it = negative.begin(); it != negative.end(); ++it){
//        context.import_raw_directory(batch_dir + *it, &analyze_fkt);
//    }
//    for(it = test_negative.begin(); it != test_negative.end(); ++it){
//        context.import_raw_directory(batch_dir + *it, &analyze_fkt);
//    }

//    double features_per_image = current_features / (double) current_count;

//    return features_per_image;
//}

//int binary_search(int min, int max, double find, robot_detection::GlobalConfig &cfg, ContextStatic &context)
//{
//    int mid = 0;
//    double features_per_image = 0;
//    while(max > min){
//        mid = (max + min) / 2;

//        cfg.extractor_threshold = mid;
//        context.apply_config(cfg);

//        features_per_image = count_features(batch_dir, context);

//        if(features_per_image > find){
//            min = mid + 1;
//        } else if(features_per_image < find) {
//            max = mid - 1;
//        } else {
//            break;
//        }

//    }

//    std::cout << "was looking for " << find << ", returned at " << features_per_image << "               <<" << std::endl;
//    return mid;
//}

//void handle_keypoint_type(int down, int up, robot_detection::GlobalConfig &cfg, ContextStatic &context, int type)
//{
//    cfg.keypoint_type = type;

//    std::cout << "max\r" << std::flush;
//    // find threshold so that count <= low
//    int max = binary_search(down, up, low, cfg, context);

//    std::cout << "min\r" << std::flush;
//    // find threshold so that count >= high
//    int min = binary_search(down, up, high, cfg, context);

//    std::cout << "\rdetector " << Extractor::write_keypoint(cfg.keypoint_type) << "\t[ " << min << " : " << max << " ]" << std::endl;
//}

//int main(int argc, char ** argv) {
//    ros::init(argc,argv, "robot_trainer_batch");

//    MatchScorerFactory::set_type(MatchScorerFactory::Scorer::HOMOGRAPHY);
//    DatabaseStrategyFactory::set_type(DatabaseStrategyFactory::Strategy::BIN);

//    robot_detection::GlobalConfig cfg = Extractor::create_default_cfg();


//    std::string cfg_path(getenv("RABOT"));
//    cfg_path += "/Config/RobotDetection/";
//    batch_dir = cfg_path + "/feature_data_training/";

//    ContextStatic context(cfg_path + "robot.db", bin_count);
//    context.apply_config(cfg);

//    std::string p = batch_dir + "/training.yaml";
//    std::ifstream ifs(p.c_str());
//    YAML::Parser parser(ifs);
//    YAML::Node doc;

//    FrameIO::MAX_IMPORT_PER_DIR = 20;

//    if(!parser.GetNextDocument(doc)){
//        ROS_ERROR_STREAM("cannot parse " << p);
//        return 1;
//    }

//    doc["positive"]            >> training;
//    doc["negative"]            >> negative;
//    doc["positive_validation"] >> validation;
//    doc["positive_test"]       >> test_positive;
//    doc["negative_test"]       >> test_negative;


//    Stopwatch w;

////    handle_keypoint_type(0, 255, cfg, context, MatcherFactory::Keypoint::BRISK); // 0
//    handle_keypoint_type(0, 1024, cfg, context, Extractor::Keypoint::SIFT); // 1
////    handle_keypoint_type(0, 3000, cfg, context, MatcherFactory::Keypoint::SURF); // 2
////    handle_keypoint_type(0, 255, cfg, context, MatcherFactory::Keypoint::ORB);  // 3
////    handle_keypoint_type(0, 255, cfg, context, MatcherFactory::Keypoint::FAST); // 4
////    handle_keypoint_type(0, 255, cfg, context, MatcherFactory::Keypoint::AGAST);// 5
////    handle_keypoint_type(0, 255, cfg, context, MatcherFactory::Keypoint::MSER); // 6
////    handle_keypoint_type(0, 255, cfg, context, MatcherFactory::Keypoint::STAR); // 7
////    handle_keypoint_type(0, 100, cfg, context, MatcherFactory::Keypoint::GFTT); // 8


//    std::cout << "found parameters in " << w.sElapsedDouble() / 60.0 << " minutes." << std::endl;

//    return 0;
//}
