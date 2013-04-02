#include "analyzer_adapter.h"

/// PROJECT
#include <analyzer/analyzer.h>

/// SYSTEM
#include <QtGui>
#include <utils/LibUtil/Stopwatch.h>

AnalyzerAdapter::AnalyzerAdapter(Analyzer& analyzer)
    : analyzer(analyzer)
{
}

AnalyzerAdapter::~AnalyzerAdapter()
{
}

void AnalyzerAdapter::runHeadless()
{
    Stopwatch sw;

    int rate = 30;
    int slot_ms = floor(1000 / (double) rate);

    while(true) {
        int dt_ms = sw.msElapsed();
        sw.reset();

        double dt = dt_ms * 1000.0;
        analyzer.tick(dt);

        usleep(slot_ms - dt_ms);
    }
}
