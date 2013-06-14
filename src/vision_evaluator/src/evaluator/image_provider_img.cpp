/// HEADER
#include "image_provider_img.h"

REGISTER_IMAGE_PROVIDERS(ImageProviderImg, jpg, png, gif, ppm, pgm)

using namespace vision_evaluator;


boost::function<bool(ImageProvider*)> ImageProviderImg::Identity
= boost::bind(&ImageProviderImg::checkIdentity, _1);

ImageProviderImg::ImageProviderImg(const std::string& path)
    : img_(cv::imread(path)), displayed(false)
{
}

ImageProvider* ImageProviderImg::createInstance(const std::string& path)
{
    return new ImageProviderImg(path);
}

bool ImageProviderImg::checkIdentity(ImageProvider* other)
{
    return dynamic_cast<ImageProviderImg*>(other) != NULL;
}

void ImageProviderImg::next(cv::Mat& img, cv::Mat& mask)
{
    displayed = true;
    img = cv::Mat(img_.rows + 80, img_.cols + 80, img_.type(), cv::Scalar::all(0));
    cv::Rect roi(40, 40, img_.cols, img_.rows);

    img_.copyTo(cv::Mat(img, roi));
}

bool ImageProviderImg::hasNext()
{
    return true;//!displayed;
}
