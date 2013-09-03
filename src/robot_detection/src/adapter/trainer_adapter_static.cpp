/// HEADER
#include "trainer_adapter_static.h"

/// PROJECT
#include <analyzer/trainer.h>
#include <data/directory_io.h>
#include <data/frame_io.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "yaml-cpp/yaml.h"

namespace bfs = boost::filesystem;

TrainerAdapterStatic::TrainerAdapterStatic(Trainer& trainer, std::ostream& out)
    : TrainerAdapter(trainer), out(out),
      db(new EvaluationStrategyDecorator(trainer.getDatabaseStrategy()))
{
    trainer.replaceDatabaseStrategy(db);
}

TrainerAdapterStatic::~TrainerAdapterStatic()
{
}

void TrainerAdapterStatic::importTest()
{
    // foreach test example -> test
    INFO("importing test examples");
    boost::function<bool(Frame::Ptr)> func_neg =
        boost::bind(&EvaluationStrategyDecorator::testFrame, db, _1, false);
    boost::function<bool(Frame::Ptr)> func_pos =
        boost::bind(&EvaluationStrategyDecorator::testFrame, db, _1, true);

    unsigned p = test_positive.size();
    unsigned n = test_negative.size();
    unsigned max = std::max(p, n);

    try {

        for(unsigned i = 0; i < max; ++i) {
            DirectoryIO pos, neg;
            if(i < p) {
                pos = DirectoryIO::import_fullsized(config("batch_dir").as<std::string>() + test_positive[i] + "/positive/", false, func_pos);
            }
            if(i < n) {
                neg = DirectoryIO::import_raw(config("batch_dir").as<std::string>() + test_negative[i], func_neg);
            }

            while(pos.next() | neg.next());
        }
    } catch(DirectoryIO::AbortException& e) {
        INFO("aborted iteration");
    }
}

namespace
{
void import_helper(boost::function<DirectoryIO(const std::string&, boost::function<bool (Frame::Ptr)> &)> dir_io,
                   const std::string& prefix, std::vector<std::string> &paths, const std::string& postfix,
                   boost::function<bool(Frame::Ptr)> callback)
{
    for(unsigned i = 0; i < paths.size(); ++i) {
        dir_io(prefix + paths[i] + postfix, callback).import_all();
    }
}
}

void TrainerAdapterStatic::importNegative()
{
    INFO("importing negative examples");
    import_helper(boost::bind(&DirectoryIO::import_raw, _1, _2), config("batch_dir"), negative, "", boost::bind(&Trainer::addNegativeExample, &trainer, _1));
}

void TrainerAdapterStatic::importPositive()
{
    INFO("importing training examples");
    import_helper(boost::bind(&DirectoryIO::import_cropped, _1, true, _2), config("batch_dir"), training, "/positive/", boost::bind(&Trainer::analyze, &trainer, _1));
}

void TrainerAdapterStatic::importValidation()
{
    INFO("importing validation examples");
    import_helper(boost::bind(&DirectoryIO::import_cropped, _1, true, _2), config("batch_dir"), validation, "/positive/", boost::bind(&Trainer::addValidationExample, &trainer, _1));
}

void TrainerAdapterStatic::import()
{
    std::string p = config("batch_dir").as<std::string>() + "/training.yaml";
    std::ifstream ifs(p.c_str());
    YAML::Parser parser(ifs);
    YAML::Node doc;

    if(!parser.GetNextDocument(doc)) {
        ERROR("cannot parse " << p);
        return;
    }

    doc["positive"]            >> training;
    doc["negative"]            >> negative;
    doc["positive_validation"] >> validation;
    doc["positive_test"]       >> test_positive;
    doc["negative_test"]       >> test_negative;


    trainer.startTraining();

    importNegative();
    importPositive();

    if(config("use_pruning")) {
        importValidation();
    }

    trainer.validate();

    trainer.stopTraining();

    importTest();

    while(db->outputTestResults(out));

    INFO("import complete");
}


void TrainerAdapterStatic::runHeadless()
{
    import();
}
