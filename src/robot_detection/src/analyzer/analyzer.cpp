/// HEADER
#include "analyzer.h"

/// PROJECT
#include <db_strategy/factory.h>
#include <utils/extractor.h>
#include <utils/matcher.h>

/// SYSTEM
#include <opencv2/opencv.hpp>

Analyzer::Analyzer()
    : db_strategy(DatabaseStrategyFactory::create()), current_frame(Frame::NULL_FRAME)
{
}

void Analyzer::replaceDatabaseStrategy(DatabaseStrategyInterface::Ptr db_strat)
{
    db_strategy = db_strat;
}

Analyzer::~Analyzer()
{
}

void Analyzer::extractFeatures(Frame::Ptr frame, cv::Rect roi)
{
    if(roi.width > 0) {
        frame->setRoi(roi);
    }

    Frame::ExtractorFunction extractor = boost::bind(&Extractor::extract, tools->getExtractor(), _1, _2, _3, _4);
    frame->extractFeatures(extractor);

//    const Matcher& matcherRef = *tools->matcher;

//    if(!context.negative_examples_.empty()) {
//        for(std::vector<Frame::Ptr >::iterator
//                it = context.negative_examples_.begin();
//                it != context.negative_examples_.end();
//                ++it) {

//            Frame::FilterFunction filter = boost::bind(&Matcher::matchAntiFilter, matcherRef, _1, (*it).get());
//            frame->filterNegativeMatches(filter);
//        }
//    }
}

bool Analyzer::analyze(Frame::Ptr frame)
{
    current_frame = frame;

    analyzeCurrentFrame(frame);

    return true;
}

void Analyzer::finalizeDebugImage()
{
    if(current_frame) {
        current_frame->finalizeDebugImage();
    }
}

void Analyzer::display_debug_image()
{
    finalizeDebugImage();

    signal_frame_analyzed();
}

void Analyzer::signal_frame_analyzed()
{
    frame_analyzed(current_frame);
}

void Analyzer::tick(double dt)
{
    tick_sig(dt);
}

Database* Analyzer::getDatabase()
{
    return db_strategy->getDatabase();
}


DatabaseStrategyInterface::Ptr Analyzer::getDatabaseStrategy()
{
    return db_strategy;
}
