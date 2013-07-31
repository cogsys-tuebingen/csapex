/// HEADER
#include "frame_io.h"

/// COMPONENT
#include "painter.h"

/// PROJECT
#include <common/global.hpp>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"

namespace bfs = boost::filesystem;

FrameIO::FrameIO()
{
}


void FrameIO::exportFrame(Frame* frame, const std::string& path, int id)
{
    if(!frame->isValid()) {
        WARN("illegal frame, aborting");
        return;
    }

    if(frame->isRoiFullFrame()) {
        WARN("nothing in frame");
        return;
    }

    std::time_t t = std::time(0);
    std::stringstream ss;
    ss << "frame_" << t << "_" << id;

    std::string feature_name = ss.str();

    std::string preview = path + feature_name + ".png";
    std::string feature_dir = path + feature_name + "/";
    std::string info_file = feature_dir + "info.yaml";

    bfs::create_directories(feature_dir);

    // write image data
    cv::imwrite(feature_dir + "img.ppm", frame->image_roi);
    cv::imwrite(feature_dir + "img_raw.ppm", frame->image_raw);
    cv::imwrite(feature_dir + "mask.ppm", frame->mask_roi);
    cv::imwrite(feature_dir + "mask_raw.ppm", frame->mask_raw);
    renderMasked(frame, preview);

    // write information
    YAML::Emitter emitter;
    emitter << 0;//frame->orientation.getW();
    emitter << 0;//frame->orientation.getX();
    emitter << 0;//frame->orientation.getY();
    emitter << 0;//frame->orientation.getZ();
    emitter << frame->orientation.toRadians();
    emitter << frame->distance;
    std::ofstream fout(info_file.c_str());
    fout << emitter.c_str();

    INFO(frame->orientation.toDegrees());

    INFO("exported frame to " << feature_dir);
}

Frame::Ptr FrameIO::importRaw(const std::string& path)
{
    Frame::Ptr frame(new Frame(cv::imread(path)));
    assert(!frame->getImage().empty());

    return frame;
}


Frame::Ptr FrameIO::convert(const cv::Mat& img)
{
    Frame::Ptr frame(new Frame(img));

    return frame;
}

Frame::Ptr FrameIO::convert(const cv::Mat& image, const cv::Mat& mask, cv::Rect roi)
{
    Frame::Ptr frame(new Frame(image, mask));

    if(roi.width > 0 && roi.height > 0) {
        frame->setRoi(roi);
    }

    return frame;
}

Frame::Ptr FrameIO::importCropped(const std::string& path, bool use_mask)
{
    return FrameIO::importSample(path, false, use_mask);
}

Frame::Ptr FrameIO::importFullsize(const std::string& path, bool use_mask)
{
    return FrameIO::importSample(path, true, use_mask);
}

Frame::Ptr FrameIO::importSample(const std::string& dir, bool full, bool use_mask)
{
    std::string path = dir + "/";


    // info yaml!!!!

    std::string img = (full? "img_raw.ppm" : "img.ppm");

    cv::Mat image = cv::imread(path + img);
    if(full && image.empty()) {
        WARN("using full, but no raw image found");
        image = cv::imread(path + "img.ppm");
    }

    if(image.empty()) {
        ERROR("the file " << path << img <<  " does not exist");
    }

    Frame::Ptr frame;

    if(!use_mask) {
        frame.reset(new Frame(image));

    } else {
        std::string mask_path = (full? "mask_raw.ppm" : "mask.ppm");

        cv::Mat mask;

        mask = cv::imread(path + mask_path);
        if(full && mask.empty()) {
            WARN("using full, but no raw mask found");
            mask = cv::imread(path + "mask.ppm");
        }
        if(mask.empty()) {
            ERROR("the file " << path << mask_path << " does not exist");
            assert(false);
        }


        assert(image.cols == mask.cols);
        assert(image.rows == mask.rows);

        if(mask.type() != CV_8U) {
            cv::cvtColor(mask, mask, CV_BGR2GRAY);
        }

        frame.reset(new Frame(image, mask));
    }
    readInfo(frame.get(), path);

    return frame;
}

void FrameIO::readInfo(Frame* frame, const std::string& path)
{
    std::string p = path + "/info.yaml";
    std::ifstream ifs(p.c_str());
    YAML::Parser parser(ifs);
    YAML::Node doc;

    if(!parser.GetNextDocument(doc)) {
        ERROR("cannot parse " << p);
        return;
    }
    double w,x,y,z, theta;
    doc >> w;
    parser.GetNextDocument(doc);
    doc >> x;
    parser.GetNextDocument(doc);
    doc >> y;
    parser.GetNextDocument(doc);
    doc >> z;
    parser.GetNextDocument(doc);
    doc >> theta;
    parser.GetNextDocument(doc);
    doc >> frame->distance;

    frame->orientation = theta;
}

void FrameIO::renderMasked(Frame* frame, const std::string& path)
{
    cv::Mat tmp;

    {
        boost::recursive_mutex::scoped_lock lock(frame->mutex);
        frame->image_roi.copyTo(tmp, frame->mask_roi);
        Painter(frame).visualizeOrientation(frame->orientation, &tmp);
    }

    cv::imwrite(path, tmp);
}

void FrameIO::renderSquareRoi(Frame* frame, const std::string& path)
{
    boost::recursive_mutex::scoped_lock lock(frame->mutex);

    cv::Rect roi_tmp = frame->roi;
    if(roi_tmp.width > roi_tmp.height) {
        roi_tmp.y -= (roi_tmp.width - roi_tmp.height) * 3.0 / 4.0;
        roi_tmp.height = roi_tmp.width;
    } else if(roi_tmp.width < roi_tmp.height) {
        roi_tmp.x -= (roi_tmp.height - roi_tmp.width) / 2.0;
        roi_tmp.width = roi_tmp.height;
    }

    if(roi_tmp.x < 0 || roi_tmp.y < 0 ||
            roi_tmp.x + roi_tmp.width >= frame->image_raw.cols ||
            roi_tmp.y + roi_tmp.height >= frame->image_raw.rows) {
        return;
    }

    cv::imwrite(path, cv::Mat(frame->image_raw, roi_tmp));
}
