/// PROJECT
#include <common/global.hpp>

/// SYSTEM
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

std::string frame;
std::string child_frame;

void map_tf(const geometry_msgs::PoseStampedConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Pose p;
    tf::poseMsgToTF(msg->pose, p);

//    p.setRotation(p.getRotation().inverse());
//    p.setOrigin(p.getOrigin() * -1);

    br.sendTransform(tf::StampedTransform(p, msg->header.stamp, frame, child_frame));
}

int main(int argc, char** argv)
{
    if(argc < 4) {
        INFO("usage: " << argv[0] << " pose-topic frame child_frame");
        return 0;
    }

    ros::init(argc,argv, "pose2tf");

    frame = argv[2];
    child_frame = argv[3];

    ros::NodeHandle n("~");
    ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped> (argv[1], 10, map_tf);

    ros::spin();
}
