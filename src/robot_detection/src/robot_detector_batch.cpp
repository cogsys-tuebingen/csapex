/// PROJECT
#include <analyzer/detector.h>
#include <adapter/detector_adapter_static.h>
#include <viz/analyzer_window.h>

int main(int argc, char** argv)
{
    Config cfg = Config::getGlobal();
    cfg.name = "Batch Detector";
    cfg.replaceGlobal();

    Detector detector;
    DetectorAdapterStatic node(detector);

//    node.test_on(cfg_path + "feature_data_training/negative");
    node.test_on(cfg.config_dir + "feature_data_training/kiste_braun/positive");

    return node.run<AnalyzerWindow>(argc, argv);
}
