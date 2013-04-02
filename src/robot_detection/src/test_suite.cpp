/// PROJECT
#include <config/config.h>
#include <data/frame_io.h>
#include <utils/extractor_factory.h>
#include <utils/matcher.h>
#include <utils/match_scorer_factory.h>

/// SYSTEM
#include <signal.h>

void siginthandler(int param)
{
    std::cout << "User pressed Ctrl+C" << std::endl;
    exit(1);
}

int main(int argc, char** argv)
{
    Frame::Ptr frame_a =
        FrameIO::importFullsize("/localhome/buck/rabot/Config/RobotDetection/feature_data_training/kiste_braun/positive/frame_1352136220_545/", true);

    Frame::Ptr frame_b =
        FrameIO::importFullsize("/localhome/buck/rabot/Config/RobotDetection/feature_data_training/kiste_grün/positive/frame_1352136085_429", false);

    cv::namedWindow("Matches");

    Config cfg = Config::getGlobal();
    cfg.setKeypointType(Types::Keypoint::ORB);
    cfg.setDescriptorType(Types::Descriptor::ORB);

    cfg.extractor_threshold = 10;
    cfg.matcher_threshold = 0.75;

    cfg.replaceGlobal();

    MatchScorerFactory::setType(MatchScorerFactory::Scorer::HOMOGRAPHY);


    while(cvGetWindowHandle("Matches") != NULL) {

        Extractor::Ptr extractor = ExtractorFactory::create(cfg.getKeypointType(), cfg.getDescriptorType());
        Matcher matcher(extractor->binary());
        MatchScorer::Ptr scorer = MatchScorerFactory::create(matcher);

        Frame::ExtractorFunction function = boost::bind(&Extractor::extract, extractor, _1, _2, _3, _4);
        frame_a->extractFeatures(function);
        frame_b->extractFeatures(function);

        std::vector<std::vector<cv::DMatch> > matches;
        matcher.match(frame_a.get(), frame_b.get(), matches);

        cv::Mat out;
        cv::drawMatches(frame_a->getImage(), frame_a->keypoints, frame_b->getImage(), frame_b->keypoints, matches, out);

        double score = scorer->calculateScore(*frame_a, *frame_b);

        std::stringstream txt;
        txt << "score: " << score;
        cv::putText(out, txt.str(), cv::Point(10, 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(0));

        cv::imshow("Matches", out);
        cv::waitKey(0);
    }

    return 0;
}
