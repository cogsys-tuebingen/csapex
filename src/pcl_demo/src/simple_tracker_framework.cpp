/// HEADER
#include "simple_tracker_framework.h"

/// SYSTEM
#include <visualization_msgs/Marker.h>

/// To get an understanding of the PCL API, take a quick look at these websites:
///   http://docs.pointclouds.org/trunk/group__sample__consensus.html
///   http://docs.pointclouds.org/trunk/classpcl_1_1_s_a_c_segmentation.html
///   http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php


SimpleTracker::SimpleTracker()
    : nh_("~")
{
    // input topic
    std::string cloud_topic = "/camera/depth_registered/points/filtered";
    nh_.param("cloud_topic", cloud_topic, cloud_topic);

    std::string goal_topic = "/local_goal";
    nh_.param("goal_topic", goal_topic, goal_topic);

    target_frame_ = "/base_link";
    nh_.param("target_frame", target_frame_, target_frame_);

    memory_frame_ = "/odom";
    nh_.param("memory_frame", memory_frame_, memory_frame_);

    sub_cloud_ = nh_.subscribe<PointCloud>(cloud_topic, 1, boost::bind(&SimpleTracker::callback, this, _1));
    pub_marker_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20, true);
    pub_plane_ = nh_.advertise<PointCloud>("plane", 1, true);
    pub_sphere_ = nh_.advertise<PointCloud>("sphere", 1, true);
    pub_balls_ = nh_.advertise<PointCloud>("balls", 1, true);
    pub_goal_pos_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic, 1, true);

    cloud_.reset(new PointCloud);

    f_ = boost::bind(&SimpleTracker::dynamicReconfigureCallback, this, _1, _2);
    server_.setCallback(f_);

    // indicates that tf should be initialized for the next cloud
    init_transformation_ = true;

    /// INITIALIZE YOUR VARIABLES
}




///////////////////////////////////////////////////////////////////////////
/// PER CLOUD CALLBACK
///////////////////////////////////////////////////////////////////////////

void SimpleTracker::callback(const PointCloud::ConstPtr& input)
{
    // initialize cloud processing
    init(input);

    // find the ground plane and extract it
    if(!findAndExtractPlane()) {
        ROS_DEBUG_STREAM("cannot find the plane");
    }

    // find all balls
    findAndExtractBalls();

    // process the found balls
    processBalls();
}





bool SimpleTracker::findAndExtractPlane()
{
    // 1. find a plane
    // 2. remove all point of that plane
    // 3. publish the extracted points

    // TAKE A LOOK AT THE LINKS PROVIDED IN THE FIRST LINES OF THIS FILE!

    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);

    for(int i=0; i < std::max(1, no_of_planes_to_extract_); ++i){
        // find segmenation
        /// Use segmenter_plane_ to segment the cloud into a plane (you need to set the input cloud and input normals)
        /// Set a minimum number of points to be used as a plane
        /// ....

        // extract all inliers and put them into 'extracted_plane'
        PointCloud::Ptr extracted_plane (new PointCloud);
        extracted_plane->header = cloud_->header;
        /// ....

        // remove the points from the point cloud
        /// ....

        // remove the normals from the normal cloud
        /// ....

        // update plane model
        plane_model_[0] = coefficients_plane->values[0];
        plane_model_[1] = coefficients_plane->values[1];
        plane_model_[2] = coefficients_plane->values[2];
        plane_model_[3] = coefficients_plane->values[3];

        // publish the plane
        pub_plane_.publish(extracted_plane);

        return true;
    }

    return false;
}

void SimpleTracker::findAndExtractBalls()
{
    pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);
    PointCloud::Ptr cloud_sphere (new PointCloud);

    cloud_sphere->header = cloud_->header;
    balls_->header = cloud_->header;

    // see: findAndExtractPlane()

    for(int i=0; i < no_of_spheres_to_extract_; ++i){
        // find segmenation
        /// ....

        // calculate sphere position
        pcl::PointXYZ sphere_center;// .... (coefficients_sphere);
        /// ....

        // update the sphere model
        // bonus: use plane model to limit ball positions to the ground
        /// ....

        // extract all inliers
        PointCloud::Ptr extracted_sphere_points (new PointCloud);
        /// ....

        // analyze the ball
        extractBall(sphere_center, extracted_sphere_points);

        // remove the points from the point cloud
        /// ....

        // remove the normals from the normal cloud
        /// ....
    }

    // publish the spheres
    pub_balls_.publish(balls_);
    pub_sphere_.publish(cloud_sphere);
}

void SimpleTracker::extractBall(pcl::PointXYZ center, PointCloud::Ptr points)
{
    // iterate all points in 'points' and calculate the mean color
    PointT ball;
    /// ball has fields: ['x', 'y', 'z', 'r', 'g', 'b']
    /// ....

    BOOST_FOREACH(PointT& center, points->points) {
        /// ....
        /// initialize 'ball' and compute it's mean color
        /// write the resulting color into 'ball''s member fields
    }

    /// ....
    // publish the new point cloud, containing all balls
    balls_->points.push_back(ball);
}

void SimpleTracker::processBalls()
{
    BOOST_FOREACH(PointT& ball, balls_->points) {
        ///if(isGreen(ball)) {
        /// ....
        ///}
    }

}







///////////////////////////////////////////////////////////////////////////
/// 30Hz callback
///////////////////////////////////////////////////////////////////////////

void SimpleTracker::tick()
{
    // fix tf errors in bag files
    ros::Time now = ros::Time::now();
    if(now < last_tf_check_) {
        tfl_.clear();
        init_transformation_ = true;
    }
    last_tf_check_ = now;

    // this code runs at a higher frequency than the cloud callback (~30Hz)
    bool send_ball_pose = false;
    /// update 'send_ball_pose' to be true, iff a ball is seen

    if(send_ball_pose) {
        PointT last_ball_odom; /// DO NOT SEND THIS EMPTY POINT, SEND THE TARGET POSE
        sendBallPoseAsGoalPose(last_ball_odom);
    } else {
        sendCarrotPose();
    }
}

void SimpleTracker::sendBallPoseAsGoalPose(PointT& ball)
{
    // transform the ball pose to the target frame and publish its pose as the goal pose
    tf::Quaternion quat;
    PointT ball_target_frame = transformPoint(ball, target_frame_, memory_frame_, &quat);

    // use the transformed pose (pt + quaternion) and publish it as the goal pose
    /// geometry_msgs::PoseStamped goal;
    /// ....
    /// pub_goal_pos_.publish(goal);
}

void SimpleTracker::sendCarrotPose()
{
    /// ....
    ///pub_goal_pos_.publish(goal);
}







///////////////////////////////////////////////////////////////////////////
/// MAIN FUNCTION
///////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_tracker");
    ROS_INFO_STREAM("starting simple tracker");

    SimpleTracker demo;

    ros::WallRate rate(30);

    while(ros::ok()) {
        demo.tick();
        ros::spinOnce();
        rate.sleep();
    }
}






///////////////////////////////////////////////////////////////////////////
/// THE FOLLOWING CODE IS GIVEN AND SHOULD NOT NEED TO BE MODIFIED:
///////////////////////////////////////////////////////////////////////////

void SimpleTracker::estimateNormals()
{
    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    normal_estimation.setSearchMethod (tree);
    normal_estimation.setInputCloud (cloud_);
    normal_estimation.setKSearch (50);
    normal_estimation.compute (*normals_);
}


void SimpleTracker::init(const PointCloud::ConstPtr& input)
{
    if(init_transformation_) {
        try {
            init_transformation_ = tfl_.waitForTransform(input->header.frame_id, target_frame_, ros::Time(0), ros::Duration(0.5));
        } catch(tf::ConnectivityException& e) {
            ROS_WARN_STREAM(e.what());
        }
    }

    normals_.reset(new pcl::PointCloud<pcl::Normal>);
    balls_.reset (new PointCloud);
    *cloud_ = *input;

    estimateNormals();
}

SimpleTracker::PointT SimpleTracker::transformPoint(const SimpleTracker::PointT& pt,const std::string& target_frame, const std::string& source_frame, tf::Quaternion* quat)
{
    tf::StampedTransform transformation;
    try {
        tfl_.lookupTransform(target_frame, source_frame, ros::Time(0), transformation);
    } catch(tf::TransformException& e) {
        ROS_ERROR_STREAM(e.what());
        tfl_.clear();
        return PointT();
    }

    tf::Vector3 ball_pt(pt.x, pt.y, pt.z);
    tf::Vector3 ball_pt_target_frame = transformation (ball_pt);

    PointT result;
    result.x = ball_pt_target_frame.x();
    result.y = ball_pt_target_frame.y();
    result.z = ball_pt_target_frame.z();

    if(quat) {
        *quat = tf::createQuaternionFromYaw(std::atan2(ball_pt_target_frame.y(), ball_pt_target_frame.x()));
    }

    return result;
}

void SimpleTracker::initializeSegmentations()
{
    segmenter_plane_.setOptimizeCoefficients (true);
    segmenter_plane_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    segmenter_plane_.setNormalDistanceWeight (0.1);
    segmenter_plane_.setMethodType (ransac_plane_);
    segmenter_plane_.setMaxIterations (iterations_plane_);
    segmenter_plane_.setDistanceThreshold (0.025);

    segmenter_sphere_.setOptimizeCoefficients (true);
    segmenter_sphere_.setModelType (pcl::SACMODEL_SPHERE);
    segmenter_sphere_.setMethodType (ransac_sphere_);
    segmenter_sphere_.setNormalDistanceWeight (sphere_normal_distance_weight_);
    segmenter_sphere_.setMaxIterations (iterations_sphere_);
    segmenter_sphere_.setDistanceThreshold (sphere_distance_threshold_);
    segmenter_sphere_.setRadiusLimits (sphere_r_min_, sphere_r_max_);
}

void SimpleTracker::dynamicReconfigureCallback(pcl_demo::pcl_demoConfig &config, uint32_t level) {
    ransac_plane_ = config.ransac_plane;
    ransac_sphere_ = config.ransac_sphere;

    sphere_r_min_ = config.sphere_radius_min;
    sphere_r_max_ = config.sphere_radius_max;

    iterations_plane_ = config.iterations_plane;
    iterations_sphere_ = config.iterations_sphere;

    sphere_min_points_ = config.sphere_min_points;

    sphere_distance_threshold_ = config.sphere_distance_threshold;
    sphere_normal_distance_weight_ = config.sphere_normal_distance_weight;

    no_of_planes_to_extract_ = config.planes;
    no_of_spheres_to_extract_ = config.spheres;


    initializeSegmentations();
}
