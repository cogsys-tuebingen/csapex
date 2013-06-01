#ifndef VIDEOWRITER_HPP
#define VIDEOWRITER_HPP

/// SYSTEM
#include <opencv2/opencv.hpp>

class ExternalVideoWriter {
public:
    ExternalVideoWriter()
        : open_(false), frame_(0)
    {}

    bool open(const std::string& filename, double fps, cv::Size frameSize) {
        open_ = true;
        filename_ = filename;

        std::stringstream ss;
        ss << filename << ".dir/frame_";
        prefix_ = ss.str();
    }

    bool isOpened() const {
        return open_;
    }

    VideoWriter& operator << (const cv::Mat& image) {
        std::stringstream file;
        file << prefix_ << frame_ << ".png";
        cv::imwrite(file.str(), image);
    }

private:
    bool open_;
    int frame_;
    std::string filename_;
    std::string prefix_;
};

#endif // VIDEOWRITER_HPP
