/// HEADER
#include "simple_tracker.h"

/// SYSTEM
#include <visualization_msgs/Marker.h>

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

    // <STUDENTS SHOULD PRODUCE>
    has_ball = false;
    max_wait_time = ros::Duration(1);
    last_tf_check_ = ros::Time::now();
    // </STUDENTS SHOULD PRODUCE>
}


void SimpleTracker::callback(const PointCloud::ConstPtr& input)
{
    // initialize cloud processing
    init(input);
    // extract normal vectors
    estimateNormals();

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

    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);

    for(int i=0; i < std::max(1, no_of_planes_to_extract_); ++i){
        // find segmenation
        segmenter_plane_.setInputCloud (cloud_);
        segmenter_plane_.setInputNormals (normals_);
        segmenter_plane_.segment (*inliers_plane, *coefficients_plane);

        if(inliers_plane->indices.size() < 100) {
            continue;
        }

        // extract all inliers
        PointCloud::Ptr extracted_plane (new PointCloud);
        extracted_plane->header = cloud_->header;

        extract_points_.setInputCloud (cloud_);
        extract_points_.setIndices (inliers_plane);
        extract_points_.setNegative (false);
        extract_points_.filter (*extracted_plane);

        // remove the points from the point cloud
        extract_points_.setNegative (true);
        extract_points_.filter (*cloud_);

        // remove the normals from the normal cloud
        extract_normals_.setNegative (true);
        extract_normals_.setInputCloud (normals_);
        extract_normals_.setIndices (inliers_plane);
        extract_normals_.filter (*normals_);

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
    pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);
    PointCloud::Ptr cloud_sphere (new PointCloud);

    cloud_sphere->header = cloud_->header;
    balls_->header = cloud_->header;

    // see: findAndExtractPlane()

    for(int i=0; i < no_of_spheres_to_extract_; ++i){
        // <STUDENTS SHOULD PRODUCE>
        // find segmenation (
        segmenter_sphere_.setInputCloud (cloud_);
        segmenter_sphere_.setInputNormals (normals_);

        segmenter_sphere_.segment (*inliers_sphere, *coefficients_sphere);

        if(inliers_sphere->indices.size() < sphere_min_points_) {
            continue;
        }

        // bonus: use plane to limit ball positions to the ground
        pcl::PointXYZ sphere_center;
        /// pcl::PointXYZ sphere_center = ... // (coefficients_sphere);
        sphere_center.x = coefficients_sphere->values[0];
        sphere_center.y = coefficients_sphere->values[1];
        sphere_center.z = coefficients_sphere->values[2];

        double dist = pcl::pointToPlaneDistanceSigned(sphere_center, plane_model_);

        if(dist < - 3 * sphere_r_max_ || dist > 3 * sphere_r_max_) {
            std::cout << "skipping ball with distance " << dist << std::endl;
            continue;
        }

        // extract all inliers
        PointCloud::Ptr extracted_sphere_points (new PointCloud);
        /// PointCloud::Ptr extracted_sphere_points (new PointCloud);
        extract_points_.setInputCloud (cloud_);
        extract_points_.setIndices (inliers_sphere);
        extract_points_.setNegative (false);
        extract_points_.filter (*extracted_sphere_points);

        *cloud_sphere += *extracted_sphere_points;

        // analyze the ball
        extractBall(sphere_center, extracted_sphere_points);
        /// extractBall(sphere_center, extracted_sphere_points);

        // remove the points from the point cloud
        extract_points_.setNegative (true);
        extract_points_.filter (*cloud_);

        // remove the normals from the normal cloud
        extract_normals_.setNegative (true);
        extract_normals_.setInputCloud (normals_);
        extract_normals_.setIndices (inliers_sphere);
        extract_normals_.filter (*normals_);
    }

    // publish the spheres
    pub_balls_.publish(balls_);
    pub_sphere_.publish(cloud_sphere);
}

void SimpleTracker::extractBall(pcl::PointXYZ center, PointCloud::Ptr points)
{
    // iterate all points in 'points' and calculate the mean color
    PointT ball;

    // <STUDENTS SHOULD PRODUCE>
    ball.x = center.x;
    ball.y = center.y;
    ball.z = center.z;
    ball.a = 255;

    int r = 0;
    int g = 0;
    int b = 0;

    double c = 0;
    BOOST_FOREACH(PointT& center, points->points) {
        r += center.r;
        g += center.g;
        b += center.b;

        ++c;
    }

    ball.r = r / c * 1.5;
    ball.g = g / c * 1.5;
    ball.b = b / c * 1.5;
    // </STUDENTS SHOULD PRODUCE>

    balls_->points.push_back(ball);
}

void SimpleTracker::processBalls()
{
    BOOST_FOREACH(PointT& ball, balls_->points) {
        if(isGreen(ball)) {
            // <STUDENTS SHOULD PRODUCE>
            has_ball = true;
            last_ball_odom = transformPoint(ball, memory_frame_, balls_->header.frame_id);
            last_ball_time = ros::Time::now();
            return;
            // </STUDENTS SHOULD PRODUCE>
        }
    }

}

/// 30Hz callback
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
    // <STUDENTS SHOULD PRODUCE>
    bool send_ball_pose = has_ball && ros::Time::now() < last_ball_time + max_wait_time;
    /// bool send_ball_pose = ...
    // </STUDENTS SHOULD PRODUCE>

    if(send_ball_pose) {
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

    // <STUDENTS SHOULD PRODUCE>
    // use the transformed pose (pt + quaternion) and publish it as the goal pose
    /// geometry_msgs::PoseStamped goal;
    /// ...
    /// pub_goal_pos_.publish(goal);
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = ball_target_frame.x;
    goal.pose.position.y = ball_target_frame.y;
    goal.pose.position.z = ball_target_frame.z;

    tf::quaternionTFToMsg(quat, goal.pose.orientation);

    goal.header.frame_id = target_frame_;

    pub_goal_pos_.publish(goal);
    // </STUDENTS SHOULD PRODUCE>
}

void SimpleTracker::sendCarrotPose()
{
    // <STUDENTS SHOULD PRODUCE>
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = 0.5;
    goal.pose.position.y = 0;
    goal.pose.position.z = 0;

    goal.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    goal.header.frame_id = target_frame_;

    pub_goal_pos_.publish(goal);
    // </STUDENTS SHOULD PRODUCE>
}


/// THESE ARE HELPER FUNCTIONS YOU NEED TO IMPLEMENT:
bool SimpleTracker::isBlue(PointT& ball)
{
    // <STUDENTS SHOULD PRODUCE>
    return ball.b > ball.g * 1.2 && ball.b > ball.r * 1.2;
    // </STUDENTS SHOULD PRODUCE>
}

bool SimpleTracker::isMagenta(PointT& ball)
{
    // <STUDENTS SHOULD PRODUCE>
    return ball.r > ball.g * 1.2 && ball.r > ball.b * 1.2;
    // </STUDENTS SHOULD PRODUCE>
}

bool SimpleTracker::isGreen(PointT& ball)
{
    // <STUDENTS SHOULD PRODUCE>
    return ball.g > ball.b * 1.2 && ball.g > ball.r * 1.2;
    // </STUDENTS SHOULD PRODUCE>
}

/// MAIN FUNCTION
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_demo_node");
    ROS_INFO_STREAM("starting PCL demo node");

    SimpleTracker demo;

    ros::WallRate rate(30);

    while(ros::ok()) {
        demo.tick();
        ros::spinOnce();
        rate.sleep();
    }
}

/// THE FOLLOWING CODE IS GIVEN AND SHOULD NOT NEED TO BE MODIFIED:
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
