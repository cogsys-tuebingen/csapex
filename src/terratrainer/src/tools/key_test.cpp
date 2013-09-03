#include <opencv2/opencv.hpp>
int main(int argc, char *argv[])
{
    while(true) {
        int key = cv::waitKey(0);
        cv::namedWindow("KeyTest");
        std::cout << key << std::endl;
    }
    return 0;
}

