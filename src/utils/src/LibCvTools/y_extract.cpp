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

    cv::cvtColor(image, image, CV_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(image, channels);

    cv::Mat scaled;
    cv::resize(channels[2], scaled, cv::Size(image.rows / 5 , image.cols / 5));
    cv::imshow("V", scaled);
    cv::waitKey(0);


    int up_limit_v = 255;
    int low_limit_v = 0;
    int up_limit_s = 255;
    int low_limit_s = 0;
    int max_v       = 255;
    int gray_1      = 0;
    int gray_2      = 0;
    cv::namedWindow("Shadow adjust");
    cv::Mat tmp_ch3 = channels[2].clone();
    cv::Mat tmp_ch2 = channels[1].clone();

    cv::createTrackbar("upper border v", "Shadow adjust",  &up_limit_v, 255);
    cv::createTrackbar("lower border v", "Shadow adjust",  &low_limit_v, 255);
    cv::createTrackbar("upper border s", "Shadow adjust",  &up_limit_s, 255);
    cv::createTrackbar("lower border s", "Shadow adjust",  &low_limit_s, 255);
    cv::createTrackbar("max v", "Shadow adjust",        &max_v, 255);
    cv::createTrackbar("gray offset 1", "Shadow adjust", &gray_1, 255);
    cv::createTrackbar("gray offset 2", "Shadow adjust", &gray_2, 255);

    bool end = false;
    while(!end) {
        cv::normalize(tmp_ch3, channels[2], up_limit_v, low_limit_v, cv::NORM_MINMAX);
        cv::normalize(tmp_ch2, channels[1], up_limit_s, low_limit_s, cv::NORM_MINMAX);
        //        channels[2].setTo(255);
        //        channels[1].setTo(255);

        cv::Mat mask;
        cv::threshold(channels[0], mask, max_v, 255, CV_THRESH_BINARY);
        //        cv::Mat gray_mat1(mask.rows, mask.cols, mask.type(), cv::Scalar::all(gray_1));
        //        cv::Mat gray_mat2(mask.rows, mask.cols, mask.type(), cv::Scalar::all(gray_2));
        //        cv::subtract(tmp_ch2,gray_mat1, channels[1], mask);
        //        cv::add(tmp_ch3,gray_mat2, channels[2], mask);
        cv::imshow("mask", mask);

        cv::Mat show;
        cv::merge(channels, show);
        cv::cvtColor(show, show, CV_HSV2BGR);

        std::vector<cv::Mat> bgr_channels;
        cv::split(show, bgr_channels);
        for(int i = 0 ; i < bgr_channels.size() ; ++i) {
            cv::Mat tmp = bgr_channels[i].clone();
            cv::equalizeHist(tmp, tmp);
            tmp.copyTo(bgr_channels[i], mask);
        }
        cv::merge(bgr_channels, show);

        cv::resize(show, show, cv::Size(1280 , 800));
        cv::imshow("test", show);
        int key = cv::waitKey(19);
        end = key == 27 || key == 1048603;
    }

    cv::imwrite(argv[2], channels[0]);

    return 0;
}
