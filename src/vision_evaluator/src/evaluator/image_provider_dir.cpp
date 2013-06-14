/// HEADER
#include "image_provider_dir.h"

namespace bfs = boost::filesystem;

REGISTER_IMAGE_PROVIDERS(ImageProviderDir, DIRECTORY)

using namespace vision_evaluator;

ImageProviderDir::ImageProviderDir(const std::string& directory)
    : is_right_format(false)//,dir_(directory), dir_it_(directory)
{
    std::cout << directory << "!!!" << std::endl;

    boost::filesystem::directory_iterator dir(directory);
    boost::filesystem::directory_iterator end;

    for(; dir != end; ++dir) {
        bfs::path path = dir->path();
        if(path.filename() == "img.ppm") {
            std::cout << "image" << std::endl;
            img_ = cv::imread(path.string());

        } else if(path.filename() == "mask.ppm") {
            std::cout << "mask" << std::endl;
            mask_ = cv::imread(path.string());
        }
    }

    if(!img_.empty() && !mask_.empty()) {
        is_right_format = true;
        std::cout << "got mask and image" << std::endl;
    }
}

ImageProviderDir::~ImageProviderDir()
{
}

ImageProvider* ImageProviderDir::createInstance(const std::string& path)
{
    return new ImageProviderDir(path);
}

bool ImageProviderDir::hasNext()
{
    return is_right_format;//bfs::exists(dir_) && dir_it_ != end_it_;
}

void ImageProviderDir::next(cv::Mat& img, cv::Mat& mask)
{
    img = cv::Mat(img_.rows + 80, img_.cols + 80, img_.type(), cv::Scalar::all(0));
    cv::Rect roi(40, 40, img_.cols, img_.rows);

    img_.copyTo(cv::Mat(img, roi));

    mask = cv::Mat(img.rows, img.cols, CV_8UC1, cv::Scalar::all(0));

    int d = 5;
    cv::rectangle(mask, cv::Rect(roi.x+d, roi.y+d, roi.width-2*d, roi.height-2*d), cv::Scalar::all(255), CV_FILLED);
}
