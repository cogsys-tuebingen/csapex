/// HEADER
#include "evaluation_strategy_decorator.h"

/// PROJECT
#include <data/frame_io.h>
#include <data/matchable_pose.h>
#include <db/bin_database.h>
#include <utils/extractor.h>
#include <utils/matcher.h>

/// SYSTEM
#include <fstream>
#include <utils/LibUtil/Stopwatch.h>

EvaluationStrategyDecorator::EvaluationStrategyDecorator(DatabaseStrategyInterface::Ptr decorated)
    : DatabaseStrategyDecorator(decorated)
{
    startEvaluation();
}

EvaluationStrategyDecorator::~EvaluationStrategyDecorator()
{
    if(roc_creator) {
        delete roc_creator;
    }
}

MatchablePose* EvaluationStrategyDecorator::test(Frame::Ptr current, RocCreator::Data& rocdata, bool positive)
{
    Stopwatch sw;

    Frame::ExtractorFunction extractor = boost::bind(&Extractor::extract, tools->getExtractor(), _1, _2, _3, _4);
    current->extractFeatures(extractor);

    int extract_time_ms = sw.msElapsed();
    summed_extract_time += extract_time_ms;

    extract_time.push_back(extract_time_ms);
    feature_count_frame.push_back(current->descriptors.rows);


    int feature_count = 0;

    sw.reset();
    cv::Rect roi(0, 0, -1, -1);
    MatchablePose* pose = decorated->detect(current, roi, &rocdata.score);
    int lookup_time_ms = sw.msElapsed();

    if(pose && !pose->keypoints.empty() && !current->keypoints.empty()) {
        try {
            cv::Mat matches;
            cv::drawMatches(pose->image, pose->keypoints, current->getImageRoi(), current->keypoints,
                            pose->last_matches, matches,
                            cv::Scalar::all(-1), cv::Scalar::all(0), std::vector<std::vector<char> >(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            rocdata.user_data = matches;
        } catch(cv::Exception& e) {
            ERROR("eval decorator: " << e.what());
        }
    } else {
        rocdata.user_data = current->getImageRoi();
    }

    if(!getDatabase()->debug.empty()) {
        cv::Mat debug = getDatabase()->debug;
        cv::Rect size(0, rocdata.user_data.rows - debug.rows / 2 - 1, debug.cols / 2, debug.rows / 2);
        cv::resize(debug, cv::Mat(rocdata.user_data, size), cv::Size(size.width, size.height));
    }

    summed_lookup_time += lookup_time_ms;
    summed_feature_count += feature_count;

    lookup_time.push_back(lookup_time_ms);
    feature_count_robot.push_back(feature_count);

    if(positive && pose != MatchablePose::NULL_POSE) {
        double yaw_ref = current->orientation.toRadians();
        double yaw_pose = pose->orientation.toRadians();

        assert(!std::isnan(yaw_pose));
        assert(!std::isnan(yaw_ref));

        rocdata.dtheta = std::abs(yaw_pose - yaw_ref);

        double diff = pose->distance - current->distance;
        rocdata.dr = std::abs(diff);

        e_theta += rocdata.dtheta;
        e_r += rocdata.dr;
    }

    return pose;
}

void EvaluationStrategyDecorator::startEvaluation()
{
    tp = 0;
    fp = 0;

    tn = 0;
    fn = 0;

    n = 0;
    p = 0;

    e_r = 0;
    e_theta = 0;

    summed_lookup_time = 0;
    summed_extract_time = 0;
    summed_feature_count = 0;

    extract_time.clear();
    lookup_time.clear();
    feature_count_frame.clear();
    feature_count_robot.clear();

    error_theta.clear();

    repeat_evaluation = false;

    roc_creator = new RocCreator(500, 20, config.getDescription());
}

bool EvaluationStrategyDecorator::testFrame(Frame::Ptr frame, bool is_positive)
{
    RocCreator::Data rocdata;
    MatchablePose* pose = test(frame, rocdata, is_positive);

    if(is_positive) {
        p++;

        if(pose == MatchablePose::NULL_POSE) {
            fn++;
        } else {
            tp++;
        }
    } else {
        n++;

        if(pose == MatchablePose::NULL_POSE) {
            tn++;
        } else {
            fp++;
        }
    }

    roc_creator->setTitle(config.getDescription());
    return roc_creator->add(rocdata, is_positive);
}

bool EvaluationStrategyDecorator::outputTestResults(std::ostream& out)
{
    e_r /= tp;
    e_theta /= tp;

    roc_creator->setTitle(config.getDescription());

    //    double e_theta_deg = e_theta / M_PI * 180;




    /// VECTOR STATS
    std::string prefix = config("result_dir").as<std::string>() + config.getDescription();

    std::ofstream fcf_out((prefix + ".stats_fcf").c_str());
    std::ofstream fcr_out((prefix + ".stats_fcr").c_str());
    std::ofstream lu_out((prefix + ".stats_lu").c_str());
    std::ofstream ex_out((prefix + ".stats_ex").c_str());
    for(int i = 0, count = feature_count_frame.size(); i < count; ++i) {
        fcf_out << feature_count_frame[i] << '\n';
        fcr_out << feature_count_robot[i] << '\n';
        lu_out << lookup_time[i] << '\n';
        ex_out << extract_time[i] << '\n';
    }

    fcf_out << std::flush;
    fcr_out << std::flush;
    lu_out << std::flush;
    ex_out << std::flush;

    /// STATS
    std::stringstream ss;
    //    ss << "tp=" << tp << ", fp=" << fp << ", tn=" << tn << ", fn=" << fn
    //       << ",p=" << positive_test_set.size() << ", n=" << negative_test_set.size()
    //       << ", e_r=" << e_r << "m, e_theta=" << e_theta_deg << "°" << std::endl;
    //    ss << "tp=" << ((100.0 * tp) / (tp + fn)) << "%, fp=" << ((100.0 * fp) / (fp + tn))
    //       << "%, tn=" << ((100.0 * tn) / (tn + fp)) << "%, fn=" << ((100.0 * fn) / (fn + tp))
    //       << "%, e_r=" << e_r << "m, e_theta=" << e_theta_deg << "°" << std::endl;
    ss << "extract_time_sum=" << summed_extract_time << "ms, avg extract="  << (summed_extract_time / ((double)(p + n))) << "ms" << std::endl;
    ss << "lookup_sum=" << summed_lookup_time << "ms, avg_lookup=" << (summed_lookup_time / ((double)(p + n))) << "ms" << std::endl;
    ss << "feature_sum=" << summed_feature_count << ", avg_feature=" << (summed_feature_count / ((double)(p + n))) << std::endl;
    //ss << "pose_count=" << db->count() << std::endl;

    out << ss.str() << std::flush;
    INFO('\n' << ss.str());


    /// ROC
    roc_creator->score_selected.connect(boost::bind(&EvaluationStrategyDecorator::setThresholdScoreCB,
                                        this, _1, _2, _3, &out));

    roc_creator->save(config("result_dir").as<std::string>() + config.getDescription() + ".png");
    if(config("interactive")) {
        roc_creator->displayInteractive();
    }
    return repeat_evaluation;
}

void EvaluationStrategyDecorator::setThresholdScoreCB(RocCreator* roc_creator, const RocCreator::Data* score, bool save, std::ostream* out)
{
    INFO("set threshold to " << score->score);
    config["score_threshold"] = score->score;
    config.replaceInstance();

    if(!score->user_data.empty()) {
        cv::Mat img = score->user_data;

        if(score->score == INFINITY) {
            cv::putText(img, "no match found", cv::Point(5, 15), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar::all(255), 5);
            cv::putText(img, "no match found", cv::Point(5, 15), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar::all(0));
        }
        cv::imshow("selected", img);
        //        cv::waitKey(33);
    }

    double tp, fp;
    roc_creator->getErrorsForThreshold(score->score, error_theta, tp, fp);

    std::stringstream ss;
    ss << "tpr=" << tp << ", fpr=" << fp << ", t=" << score->score << std::endl;

    if(out) {
        (*out) << ss.str();
    }

    if(save) {
        std::ofstream etheta((config("result_dir").as<std::string>() + config.getDescription() + ".stats_etheta").c_str());
        for(int i = 0, count = error_theta.size(); i < count; ++i) {
            etheta << error_theta[i] << '\n';
        }
        etheta << std::flush;

        EvaluationStrategyDecorator::save();
    }
}

