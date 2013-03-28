#ifndef IMAGE_PROVIDER_IMG_H
#define IMAGE_PROVIDER_IMG_H

/// COMPONENT
#include "image_provider.h"

/// SYSTEM
#include <opencv2/opencv.hpp>

namespace vision_evaluator
{

class ImageProviderImg : public ImageProvider
{
    Q_OBJECT

protected:
    ImageProviderImg(const std::string& path);

public:
    static boost::function<bool(ImageProvider*)> Identity;
private:
    static bool checkIdentity(ImageProvider*);

public:
    static ImageProvider* createInstance(const std::string& path);

    bool hasNext();
    void next();

private:
    cv::Mat img;
    bool displayed;
};

} /// NAMESPACE

#endif // IMAGE_PROVIDER_IMG_H
