///// PROJECT
//#include <analyzer/context_static.h>
//#include <data/matchable.h>
//#include <data/frame.h>
//#include <db_strategy/factory.h>
//#include <robot_detection/GlobalConfig.h>
//#include <tools/match_scorer_factory.h>
//#include <tools/extractor.h>

///// SYSTEM
//#include <boost/filesystem.hpp>
//#include <fstream>

//namespace bfs = boost::filesystem;

//ContextStatic context("robot.db", 1);

//std::map<std::string, std::map<int, std::map<int, std::map<int, int> > > > table;

//void callback(std::vector<boost::shared_ptr<Frame> >& vec, int key, int threshold, const std::string &type)
//{
//    for(std::vector<boost::shared_ptr<Frame> >::iterator it = vec.begin(); it != vec.end(); ++it){
//        table[type][threshold][key][round((*it)->distance)] = (*it)->keypoints.size();
//    }

//    vec.clear();
//}

//int main(int argc, char ** argv) {
//    ros::init(argc,argv, "stats_feat_dist");

//    std::vector<std::pair<std::string, std::string> > test;
//    test.push_back(std::pair<std::string, std::string>("front", "/home/buck/Dropbox/robot_detection/distances/front/"));
//    test.push_back(std::pair<std::string, std::string>("back", "/home/buck/Dropbox/robot_detection/distances/back/"));
//    test.push_back(std::pair<std::string, std::string>("side", "/home/buck/Dropbox/robot_detection/distances/side/"));



//    MatchScorerFactory::set_type(MatchScorerFactory::Scorer::HOMOGRAPHY);
//    DatabaseStrategyFactory::set_type(DatabaseStrategyFactory::Strategy::BIN);

//    robot_detection::GlobalConfig cfg = Extractor::create_default_cfg();

//    int min_threshold = 0;
//    int max_threshold = 100;
//    int delta = 1;

//    std::vector<std::pair<std::string, std::string> >::iterator dir_it;

//    // initialize table
//    for(dir_it = test.begin(); dir_it != test.end(); ++dir_it){
//        for(int threshold = min_threshold; threshold <= max_threshold; threshold += delta){
//            for(int key = 0; key < Extractor::Keypoint::COUNT; ++key){
//                for(int d = 1; d <= 15; ++d){
//                    table[dir_it->first][threshold][key][d] = -1;
//                }
//            }
//        }
//    }

//    // generate table content
//    for(dir_it = test.begin(); dir_it != test.end(); ++dir_it){
//        for(int threshold = min_threshold; threshold <= max_threshold; threshold += delta){
//            cfg.extractor_threshold = threshold;
//            for(int key = 0; key < Extractor::Keypoint::COUNT; ++key){
//                boost::function<void(std::vector<boost::shared_ptr<Frame> >&)> cb = boost::bind(&callback, _1, key, threshold, dir_it->first);

//                std::cout << "now testing keypoint type " << key << std::endl;

//                cfg.keypoint_type = key;
//                context.apply_config(cfg);

//                context.import_directory(dir_it->second, true, &cb);
//            }
//        }
//    }
//    // write csv file
//    for(int threshold = min_threshold; threshold <= max_threshold; threshold += delta){

//        for(dir_it = test.begin(); dir_it != test.end(); ++dir_it){
//            std::stringstream ss;
//            ss << dir_it->first << "-" << threshold << ".csv";
//            std::ofstream out(ss.str().c_str());

//            out << "\"distance\",";
//            for(int key = 0; key < Extractor::Keypoint::COUNT; ++key){
//                out << "\"" << Extractor::write_keypoint(key) << "\",";
//            }
//            out << "\n";
//            for(int d = 1; d <= 15; ++d){
//                out << d << ",";
//                for(int key = 0; key < Extractor::Keypoint::COUNT; ++key){
//                    out << table[dir_it->first][threshold][key][d] << ",";
//                }
//                out << "\n";
//            }
//            out.flush();
//            out.close();
//        }
//    }


//    return 0;
//}

