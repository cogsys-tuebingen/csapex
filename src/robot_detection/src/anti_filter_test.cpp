/// PROJECT
#include <data/frame_io.h>
#include <robot_detection/GlobalConfig.h>
#include <tools/extractor.h>
#include <tools/matcher.h>
#include <viz/database_viewer.h>

/// SYSTEM
#include <signal.h>

void siginthandler(int param)
{
    printf("User pressed Ctrl+C\n");
    exit(1);
}

int main(int argc, char** argv)
{
    if(argc != 6) {
        ROS_FATAL_STREAM("usage: " << argv[0]
                         << " <keypoint-type> <descriptor-type> <extractor-threshold> <background-img> <test-img>");
        std::stringstream ss;
        for(int i = 0; i < argc; ++i) {
            ss << argv[i] << " ";
        }
        std::string s = ss.str();
        ROS_FATAL_STREAM("       you specified " << argc << " items : " << s);
        exit(1);
    }

    robot_detection::GlobalConfig config = Extractor::create_default_cfg();

    std::string keypoint = argv[1];
    std::string descriptor = argv[2];
    config.extractor_threshold = atoi(argv[3]);
    std::string background = argv[4];
    std::string test = argv[5];

    Extractor::Keypoint::Type keypoint_type = Extractor::read_keypoint(keypoint);
    Extractor::Descriptor::Type descriptor_type = Extractor::read_descriptor(descriptor);
    config.keypoint_type = keypoint_type;
    config.descriptor_type = descriptor_type;

    Extractor extractor(config);
    Matcher matcher(extractor);

    // read image and make keypoints
    boost::shared_ptr<Frame> bg_frame = FrameIO::import_raw_from_path(background);
    boost::shared_ptr<Frame> test_frame = FrameIO::import_raw_from_path(test);

    bg_frame->extract_features(&extractor);
    test_frame->extract_features(&extractor);

    // visualize keypoints
    bg_frame->draw_keypoints_roi(cv::Scalar(0));
    test_frame->draw_keypoints_roi(cv::Scalar(0));

    // matches
    cv::Mat matches;
    std::vector<std::vector<cv::DMatch> > match;
    matcher.match(bg_frame.get(), test_frame.get(), match);
    cv::drawMatches(bg_frame->get_image(), bg_frame->keypoints, test_frame->get_image(), test_frame->keypoints, match, matches,
                    cv::Scalar::all(-1), cv::Scalar::all(0), std::vector<std::vector<char> >(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

//    cv::resize(matches, matches, cv::Size(), 0.5, 0.5);

    // remove matching background features
    matcher.anti_filter_match(test_frame.get(), bg_frame.get());

    ROS_INFO_STREAM("remaining keypoints: " << test_frame->keypoints.size());

    // get the debug image
    boost::mutex mutex;
    cv::Mat bg_img, test_img;
    test_frame->draw_keypoints_roi_highlight(test_frame->keypoints, cv::Scalar(0,0,255));
    bg_frame->finalize_debug_image(mutex);
    test_frame->finalize_debug_image(mutex);
    test_frame->get_debug_image(mutex, test_img);
    bg_frame->get_debug_image(mutex, bg_img);

    // display *remaining* features
//    cv::imshow("background", bg_img);
    cv::imshow("matches", matches);
    cv::imshow("test", test_img);

    int key = cv::waitKey(0);
    while(key == 65513) { // allow alt to move windows
        key = cv::waitKey(0);
    }

    return 0;
}
