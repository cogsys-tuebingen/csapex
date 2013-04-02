/// SYSTEM
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>

tf::TransformBroadcaster* tfb;

void callback(const tf::tfMessageConstPtr& msg, const std::string& prefix)
{
    int n = msg->transforms.size();
    for(int i=0; i < n; ++i) {
        geometry_msgs::TransformStamped t = msg->transforms[i];
        if(t.child_frame_id[0] != '/') {
            t.child_frame_id = prefix + "/" + t.child_frame_id;
        } else {
            t.child_frame_id = prefix + t.child_frame_id;
        }
        if(t.header.frame_id[0] != '/') {
            t.header.frame_id = prefix + "/" + t.header.frame_id;
        } else {
            t.header.frame_id = prefix + t.header.frame_id;
        }
        tfb->sendTransform(t);
    }

    ROS_INFO_STREAM_THROTTLE(1, "imported tf into " + prefix);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "tf_importer");

    ros::NodeHandle nh("~");

    tfb = new tf::TransformBroadcaster;

    ros::Subscriber sub = nh.subscribe<tf::tfMessage>
                          ("/robot_1/tf", 10, boost::bind(&callback, _1, "/robot_1"));

    ros::spin();

    delete tfb;
}
