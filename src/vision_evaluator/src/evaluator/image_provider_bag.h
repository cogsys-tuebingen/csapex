#ifndef IMAGE_PROVIDER_BAG_H
#define IMAGE_PROVIDER_BAG_H

/// COMPONENT
#include "image_provider_set.h"

/// SYSTEM
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace vision_evaluator
{

class ImageProviderBag : public ImageProviderSet
{
    Q_OBJECT

protected:
    ImageProviderBag(const std::string& file);

public:
    static ImageProvider* createInstance(const std::string& path);

    virtual bool hasNext();
    virtual void reallyNext(cv::Mat& img, cv::Mat& mask);

    void doInit();

private:
    void display();

private:
    std::string file_;

    rosbag::Bag bag;
    rosbag::View* view_;
    rosbag::View::iterator view_it;

    bool initiated;
};

} /// NAMESPACE

#endif // IMAGE_PROVIDER_BAG_H
