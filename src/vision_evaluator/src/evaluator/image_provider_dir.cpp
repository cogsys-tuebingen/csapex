/// HEADER
#include "image_provider_dir.h"

namespace bfs = boost::filesystem;

REGISTER_IMAGE_PROVIDERS(ImageProviderDir, DIRECTORY)

using namespace vision_evaluator;

ImageProviderDir::ImageProviderDir(const std::string& directory)
    : ImageProvider("training"), is_right_format(false)//,dir_(directory), dir_it_(directory)
{
    std::cout << directory << "!!!" << std::endl;

    boost::filesystem::directory_iterator dir(directory);
    boost::filesystem::directory_iterator end;

    for(; dir != end; ++dir) {
        bfs::path path = dir->path();
        if(path.filename() == "img.ppm") {
            std::cout << "image" << std::endl;
            img = cv::imread(path.string());

        } else if(path.filename() == "mask.ppm") {
            std::cout << "mask" << std::endl;
            mask = cv::imread(path.string());
        }
    }

    if(!img.empty() && !mask.empty()) {
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

void ImageProviderDir::next()
{
    cv::Mat grown(img.rows + 80, img.cols + 80, img.type(), cv::Scalar::all(0));
    cv::Rect roi(40, 40, img.cols, img.rows);

    img.copyTo(cv::Mat(grown, roi));

    cv::Mat mask(grown.rows, grown.cols, CV_8UC1, cv::Scalar::all(0));

    int d = 5;
    cv::rectangle(mask, cv::Rect(roi.x+d, roi.y+d, roi.width-2*d, roi.height-2*d), cv::Scalar::all(255), CV_FILLED);

    new_image(grown, mask);
}
