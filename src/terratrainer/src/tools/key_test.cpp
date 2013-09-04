#include <opencv2/opencv.hpp>
int main(int argc, char *argv[])
{
    cv::Mat test(4,4, CV_32FC1);
    cv::Mat test2(4,4, CV_8UC1);

    cv::Scalar value_to_set(22.0, 0, 0, 0);
    cv::Vec2b  value_to_set2(22, 22);

    test.col(1).setTo(value_to_set2);
    test2.col(1).setTo(value_to_set2);

    std::cout << test.at<float>(0,0) << " " << test.at<float>(0,1) << std::endl;
    std::cout << test.at<float>(0,0) << " " << test.at<float>(0,1) << std::endl;
    std::cout << (int) test2.at<uchar>(1,0) << " " << (int) test2.at<uchar>(1,1) << std::endl;
    std::cout << (int) test2.at<uchar>(1,0) << " " << (int) test2.at<uchar>(1,1) << std::endl;


    while(true) {
        int key = cv::waitKey(0);
        cv::namedWindow("KeyTest");
        std::cout << key << std::endl;
    }
    return 0;
}

