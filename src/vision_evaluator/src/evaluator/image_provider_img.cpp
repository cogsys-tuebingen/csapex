/// HEADER
#include "image_provider_img.h"

REGISTER_IMAGE_PROVIDERS(ImageProviderImg, jpg, png, gif, ppm, pgm)

using namespace vision_evaluator;


boost::function<bool(ImageProvider*)> ImageProviderImg::Identity
= boost::bind(&ImageProviderImg::checkIdentity, _1);

ImageProviderImg::ImageProviderImg(const std::string& path)
    : img(cv::imread(path)), displayed(false)
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

void ImageProviderImg::next()
{
    displayed = true;
    cv::Mat grown(img.rows + 80, img.cols + 80, img.type(), cv::Scalar::all(0));
    cv::Rect roi(40, 40, img.cols, img.rows);

    img.copyTo(cv::Mat(grown, roi));

    new_image(grown, cv::Mat());
}

bool ImageProviderImg::hasNext()
{
    return true;//!displayed;
}
