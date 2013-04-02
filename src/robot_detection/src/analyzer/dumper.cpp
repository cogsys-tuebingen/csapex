/// HEADER
#include "dumper.h"

/// PROJECT
#include <data/frame_io.h>
#include <db_strategy/db_strategy.h>

/// SYSTEM
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

Dumper::Dumper()
    : dump_vj_data(false), dump_feature_data(false), dump_reference_images(false), feature(0)
{
    state = TRAINING;
}

void Dumper::stopTraining()
{
    Trainer::stopTraining();

    if(dump_reference_images) {
        bfs::create_directories(config.ref_dir);
        db_strategy->dumpReference(config.ref_dir);
    }
}

void Dumper::training(Frame::Ptr frame)
{
    Trainer::training(frame);

    int area = frame->getRoiWidth() * frame->getRoiHeight();
    if(area > 20 * 20) {
        dumpCurrentRoiAsImage();
    }
}

void Dumper::dumpCurrentRoiAsImage()
{
    if(dump_feature_data) {
        INFO("dump");
        FrameIO::exportFrame(current_frame.get(), config.batch_dir, feature++);
    }
}
