/*
 * merge_test.cpp
 *
 *  Created on: Mar 19, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

/// PROJECT
#include <config/config.h>
#include <utils/extractor_factory.h>
#include <utils/matcher.h>
#include <utils/match_scorer_factory.h>

/// SYSTEM
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem3;

int main(int argc, char** argv)
{
    if(argc <= 1) {
        std::cout << "usage: " << argv[0] << " <path-to-test-files>" << std::endl;
        return 1;
    }

    bfs::path path = argv[1];
    if(!bfs::exists(path)) {
        std::cout << "path " << path << " does not exist" << std::endl;
        return 2;
    }

    Config cfg = Config::getGlobal();
    cfg.setKeypointType(Types::Keypoint::ORB);
    cfg.setDescriptorType(Types::Descriptor::ORB);

    cfg.extractor_threshold = 10;
    cfg.matcher_threshold = 0.75;

    cfg.replaceGlobal();

    MatchScorerFactory::setType(MatchScorerFactory::Scorer::HOMOGRAPHY);

    Extractor::Ptr extractor = ExtractorFactory::create(cfg.getKeypointType(), cfg.getDescriptorType());
    Matcher matcher(extractor->binary());

    int files = 0;

    bfs::directory_iterator end;
    for(bfs::directory_iterator dir(path); dir != end; ++dir) {
        bfs::path path = dir->path();
        if(path.extension() == ".png") {
            files++;
        }
    }

    cv::Mat images[files];
    int image = 0;
    for(bfs::directory_iterator dir(path); dir != end; ++dir) {
        bfs::path path = dir->path();
        if(path.extension() == ".png") {
            images[image] = cv::imread(path.string().c_str());
            image++;
        }
    }

    for(int i = 0; i < files; ++i) {
        cv::imshow("test", images[i]);
        cv::waitKey(1000);
    }
}
