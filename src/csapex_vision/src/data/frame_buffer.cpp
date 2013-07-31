/// HEADER
#include "frame_buffer.h"

std::vector<cv::Mat> FrameBuffer::extra_debug_img;
std::vector<std::string> FrameBuffer::extra_name;

boost::recursive_mutex FrameBuffer::extra_mutex;

FrameBuffer::FrameBuffer()
{
}

void FrameBuffer::setImage(int idx, const std::string& name, cv::Mat& img)
{
    boost::recursive_mutex::scoped_lock lock(extra_mutex);

    while(idx >= (int) extra_debug_img.size()) {
        extra_name.push_back("");
        extra_debug_img.push_back(cv::Mat());
    }
    extra_name[idx] = name;
    extra_debug_img[idx] = img;
}

void FrameBuffer::setImageAside(int idx, const std::string& name, cv::Mat left, cv::Mat& img)
{
    if(left.empty()) {
        ERROR("calling debug image aside with an empty image");
        return;
    }

    boost::recursive_mutex::scoped_lock lock(extra_mutex);

    assert(left.type() == img.type());

    int w = 400;

    int rows = std::max(img.rows, left.rows);
    int cols = 400 + left.cols;

    cv::Mat merged(rows, cols, left.type(), cv::Scalar(0));

    cv::Mat sub_left(merged, cv::Rect(0, 0, std::min(w, img.cols), img.rows));
    cv::Mat sub_right(merged, cv::Rect(w, 0, left.cols, left.rows));

    left.copyTo(sub_right);
    img.copyTo(sub_left);

    setImage(idx, name, merged);

}

int FrameBuffer::getImageCount()
{
    boost::recursive_mutex::scoped_lock lock(extra_mutex);

    return extra_debug_img.size();
}

std::string FrameBuffer::getName(int index)
{
    boost::recursive_mutex::scoped_lock lock(extra_mutex);

    if(index >= (int) extra_name.size()) {
        ERROR("extra name: index overflow " << index);

        return "";
    }
    return extra_name[index];
}

cv::Mat FrameBuffer::getImage(int id)
{
    boost::recursive_mutex::scoped_lock lock(extra_mutex);

    if(id >= (int) extra_debug_img.size()) {
        ERROR("extra image: index overflow " << id);

        return cv::Mat();
    }

    cv::Mat img;
    extra_debug_img[id].copyTo(img);
    return img;
}
