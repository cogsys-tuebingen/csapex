/// HEADER
#include "image_provider_dir.h"

/// SYSTEM
#include <boost/assign.hpp>
#include <csapex/utility/register_apex_plugin.h>

CSAPEX_REGISTER_CLASS(csapex::ImageProviderDir, csapex::MessageProvider)


namespace bfs = boost::filesystem;

using namespace csapex;

ImageProviderDir::ImageProviderDir()
    : is_right_format(false)
{
}

void ImageProviderDir::load(const std::string& directory)
{
    boost::filesystem::directory_iterator dir(directory);
    boost::filesystem::directory_iterator end;

    for(; dir != end; ++dir) {
        bfs::path path = dir->path();
        if(path.filename() == "img.ppm") {
            std::cout << "image" << std::endl;
            img_ = cv::imread(path.string(), CV_LOAD_IMAGE_UNCHANGED);

        } else if(path.filename() == "mask.ppm") {
            std::cout << "mask" << std::endl;
            mask_ = cv::imread(path.string(), CV_LOAD_IMAGE_GRAYSCALE);
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

std::vector<std::string> ImageProviderDir::getExtensions() const
{
    return boost::assign::list_of<std::string> (".DIRECTORY");
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
