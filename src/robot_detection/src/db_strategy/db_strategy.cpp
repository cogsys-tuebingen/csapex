/// HEADER
#include "db_strategy.h"

/// PROJECT
#include <data/matchable_pose.h>
#include <data/painter.h>
#include <db/database_io.h>

/// SYSTEM
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/if.hpp>
#include <utils/LibUtil/Stopwatch.h>

using namespace boost::lambda;
namespace lam = boost::lambda;

lam::placeholder1_type X;
lam::placeholder2_type Y;
lam::placeholder3_type Z;


DatabaseStrategy::DatabaseStrategy(Database* db)
    : DatabaseStrategyInterface(db)
{
}

DatabaseStrategy::~DatabaseStrategy()
{
    if(db != NULL) {
        delete db;
        db = NULL;
    }
}

bool DatabaseStrategy::load()
{
    Database* db_old = db;
    db = NULL;

    bool result = DatabaseIO::load(config("db_file"), db);

    if(db != NULL) {
        delete db_old;

    } else {
        db = db_old;
    }
    return result;
}

bool DatabaseStrategy::save()
{
    return DatabaseIO::save(config("db_file"), db);
}

void DatabaseStrategy::clear()
{
    db->clear();
}

bool DatabaseStrategy::loadConfig()
{
    Config output = Config::instance();

    bool loadOk = DatabaseIO::loadConfig(config("db_file"), output);

    if(loadOk) {
        output.replaceInstance();

        WARN(config("db_file").as<std::string>());
    }

    return loadOk;
}


MatchablePose* DatabaseStrategy::detect(Frame::Ptr frame, cv::Rect& out_roi, double* score_out)
{
    Painter(frame.get()).drawKeypointsRoi(cv::Scalar(0,0,0));

    Stopwatch sw;
    double score;
    MatchablePose* pose = db->getBestMatch(frame.get(), &score);
    sw.stop();

    if(score_out != NULL) {
        *score_out = score;
    }

    INFO("lookup time: " << sw.usElapsed() / 1e3 << "ms");

    out_roi = pose->last_roi;

    return pose;
}


void DatabaseStrategy::addFrame(Frame::Ptr f, double scale)
{
    assert(f->isValid());
    MatchablePose* p;
    if(std::abs(scale - 1.0) < 0.0001) {
        p = new MatchablePose(f);
    } else {
        Frame::Ptr tmp = f->getScaledCopy(scale);
        p = new MatchablePose(tmp);
        // tmp no longer needed, deep copied
    }

    db->add(p);
}


MatchablePose* DatabaseStrategy::getPoseByAngle(const double yaw, int* /*index*/) const
{
    double closestDistance = INFINITY;
    MatchablePose* closestPose = MatchablePose::NULL_POSE;

    // lambda pose. abs(yaw - yaw(pose.orientation))
    boost::function<double(MatchablePose*)> distanceOf =
        (lam::bind(static_cast<double(*)(const double)> (&std::abs),
                   (yaw - (lam::bind(&Angle::toRadians, (X ->* &MatchablePose::orientation))))));

    // lambda level pose. ()
    boost::function<void (int, const std::string&)> cb = (X+=0);

    // lambda level pose. if(distanceOf(pose) < closestDistance, (closestDistance = distanceOf(pose), closestPose = pose))
    boost::function<void (int, MatchablePose*)> lb =
        (if_then(lam::bind(distanceOf, Y) < var(closestDistance),
                 (var(closestDistance) = lam::bind(distanceOf, Y), var(closestPose) = Y)));

    db->traversePoses(cb, lb);

    return closestPose;
}

void DatabaseStrategy::dumpReference(const std::string& /*path*/)
{
    ERROR("reimplement");
    throw;
//    for(int i = 0, n = db->count(); i < n; ++i) {
//        std::stringstream ss;
//        Pose* pose = db->getPose(i);
//        INFO( "dump reference to " << ss.str() );
//        cv::imwrite(ss.str(), pose->image);
//    }
}
