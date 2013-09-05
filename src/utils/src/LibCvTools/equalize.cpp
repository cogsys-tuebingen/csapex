#include <opencv2/opencv.hpp>
int main(int argc, char *argv[])
{
    if(argc < 3) {
        std::cerr << "Arguments : <input image> <output image>" << std::endl;
        return 1;
    }

    cv::Mat image = cv::imread(argv[1]);
    std::vector<cv::Mat> channels;
    cv::split(image, channels);

    if(image.empty()) {
        std::cerr << "Couldn't load image!" << std::endl;
        return 1;
    }

    for(std::vector<cv::Mat>::iterator it = channels.begin() ; it != channels.end() ; ++it)
        cv::equalizeHist(*it, *it);

    cv::merge(channels, image);

    if(cv::imwrite(argv[2], image))
        std::cout << "Congrats you now own a histogram equalized image!" << std::endl;

    return 0;
}

