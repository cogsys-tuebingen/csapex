/// HEADER
#include "trainer_online.h"

/// COMPONENT
#include "detector.h"

TrainerOnline::TrainerOnline()
    : detector(new Detector)
{
    detector->frame_analyzed.connect(frame_analyzed);
}

TrainerOnline::~TrainerOnline()
{
    delete detector;
}

void TrainerOnline::stopped(Frame::Ptr frame)
{
    Trainer::stopped(frame);

    detector->analyzeCurrentFrame(frame);

    signal_frame_analyzed();
}

void TrainerOnline::tick(double dt)
{
    Trainer::tick(dt);

    detector->tick(dt);
}

