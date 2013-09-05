#include <opencv2/opencv.hpp>

int main(int argc, char *argv[])
{
    if(argc < 3) {
        std::cerr << "Arguments : <input image> <output image>" << std::endl;
        return 1;
    }


    cv::Mat image = cv::imread(argv[1]);
    if(image.empty()) {
        std::cerr << "Couldn't load image!" << std::endl;
        return 1;
    }

    cv::cvtColor(image, image, CV_BGR2YUV);
    std::vector<cv::Mat> channels;
    cv::split(image, channels);

    cv::Mat scaled;
    cv::resize(channels[0], scaled, cv::Size(image.rows / 5 , image.cols / 5));
    cv::imshow("Y", scaled);
    cv::waitKey(0);


//    int up_limit = 255;
//    int low_limit = 0;
//    cv::namedWindow("y adjust");
//    cv::Mat tmp_ch1 = channels[0].clone();

//    cv::createTrackbar("upper border y", "y adjust",  &up_limit, 255);
//    cv::createTrackbar("lower border y", "y adjust",  &low_limit, 255);
//    bool end = false;
//    while(!end) {
//        cv::normalize(tmp_ch1, channels[0], up_limit, low_limit, cv::NORM_MINMAX);
//        cv::Mat show;
//        cv::merge(channels, show);
//        cv::resize(show, show, cv::Size(1280 , 800));
//        cv::cvtColor(show, show, CV_YUV2BGR);
//        cv::imshow("test", show);
//        int key = cv::waitKey(19);
//        end = key == 27 || key == 1048603;
//    }

    cv::imwrite(argv[2], channels[0]);

    return 0;
}
