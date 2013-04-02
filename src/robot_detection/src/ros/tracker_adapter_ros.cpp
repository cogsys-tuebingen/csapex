/// HEADER
#include "tracker_adapter_ros.h"

/// PROJECT
#include <data/angle.h>
#include <data/pose.h>

/// SYSTEM
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

TrackerAdapterRos::TrackerAdapterRos(Tracker& tracker)
    : nh("~"), tracker(tracker), publish_rate_(10), last_publish_(ros::WallTime::now())
{
    tracking_frame_id_ = "/map";
    nh.param("tracking_frame", tracking_frame_id_, tracking_frame_id_);

    std::string poses_topic = "/robot_detector/poses";
    std::string hypotheses_topic = "hypotheses";
    std::string covariances_topic = "covariances_topic";

    nh.param("pose_topic", poses_topic, poses_topic);
    nh.param("hypotheses_topic", hypotheses_topic, hypotheses_topic);
    nh.param("covariances_topic", covariances_topic, covariances_topic);

    measurements_sub = nh.subscribe<geometry_msgs::PoseArray>
                       (poses_topic, 5, boost::bind(&TrackerAdapterRos::measurementCallback, this, _1));

    hypo_pub = nh.advertise<geometry_msgs::PoseArray>
               (hypotheses_topic, 1, true);
    cov_pub = nh.advertise<visualization_msgs::MarkerArray>
              (covariances_topic, 1, true);
}

void TrackerAdapterRos::measurementCallback(const geometry_msgs::PoseArrayConstPtr& msg)
{
    if(!tfl.waitForTransform(tracking_frame_id_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.01))) {
        WARN("cannot lookup transform");
        return;
    }

    tf::StampedTransform transform;
    tfl.lookupTransform(tracking_frame_id_, msg->header.frame_id, msg->header.stamp, transform);

    for(std::vector<geometry_msgs::Pose>::const_iterator it = msg->poses.begin(); it != msg->poses.end(); ++it) {
        tf::Pose tf_pose;
        tf::poseMsgToTF(*it, tf_pose);

        tf::Vector3 translation = transform(tf_pose.getOrigin());
        tf::Quaternion orientation = transform * tf_pose.getRotation();

        Pose pose;

        tf::vectorTFToEigen(translation, pose.position);
        tf::quaternionTFToEigen(orientation, pose.orientation);

        double r, p, yaw;
        Angle::quatToRPY(pose.orientation, r, p, yaw);

        tracker.update(pose);
    }
}

void TrackerAdapterRos::addHypothesisToMarkerArray(visualization_msgs::MarkerArray& markers, geometry_msgs::PoseArray& poses, const Hypothesis& hypo, bool is_valid)
{
    Eigen::VectorXd x = hypo.getState();
    Eigen::MatrixXd cov = hypo.getCovariance();

    tf::Quaternion q = tf::createQuaternionFromYaw(x(2));
    visualization_msgs::Marker marker;

    marker.header.frame_id = tracking_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "cov";
    marker.type = visualization_msgs::Marker::CYLINDER;

    if(is_valid) {
        marker.color.r = 0.1;
        marker.color.g = 1;
    } else {
        marker.color.r = 1;
        marker.color.g = 0.5;
    }

    marker.color.b = 0.2;
    marker.color.a = 0.2;

    marker.scale.x = std::sqrt(cov(0,0));
    marker.scale.y = std::sqrt(cov(1,1));
    marker.scale.z = 0.1;

    marker.id = nextMarkerId++;

    marker.lifetime = ros::Duration(1);

    marker.pose.position.x = x(0);
    marker.pose.position.y = x(1);
    tf::quaternionTFToMsg(q, marker.pose.orientation);

    poses.poses.push_back(marker.pose);

    marker.pose.position.z = -0.4;
    markers.markers.push_back(marker);
}

void TrackerAdapterRos::publishHypotheses()
{
    nextMarkerId = 0;

    visualization_msgs::MarkerArray markers;
    geometry_msgs::PoseArray poses;

    poses.header.frame_id = tracking_frame_id_;
    poses.header.stamp = ros::Time::now();

    Tracker::HypothesisIterator iterator = boost::bind(&TrackerAdapterRos::addHypothesisToMarkerArray, this, boost::ref(markers), boost::ref(poses), _1, _2);
    tracker.forEachHypothesis(iterator);

    hypo_pub.publish(poses);
    cov_pub.publish(markers);
}

int TrackerAdapterRos::run(int argc, char** argv)
{
    ros::WallRate rate(30);

    ros::WallTime last(ros::WallTime::now());

    while(ros::ok()) {
        ros::WallTime now = ros::WallTime::now();
        double dt = (now - last).toSec();
        last = now;

        tracker.predict(dt);

        if(last_publish_ + publish_rate_.expectedCycleTime() < now) {
            last_publish_ = now;
            publishHypotheses();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
