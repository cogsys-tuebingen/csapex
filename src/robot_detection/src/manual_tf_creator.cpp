/// COMPONENT
#include <robot_detection/TfCreatorConfig.h>

/// SYSTEM
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>
#include <tf/tf.h>

class ManualTFCreator
{

public:
    ManualTFCreator() {
        ros::NodeHandle nh("~");

        f = boost::bind(&ManualTFCreator::dynamic_reconfigure_callback, this, _1, _2);
        server.setCallback(f);

        nh.param("x", x, 0.0);
        nh.param("y", y, 0.0);
        nh.param("theta", theta, 0.0);

        from = "/robot_0/map";
        to = "/robot_1/map";

        nh.param("frame/from", from, from);
        nh.param("frame/to", to, to);
    }

    void dynamic_reconfigure_callback(robot_detection::TfCreatorConfig& config, uint32_t level) {
        x = config.dx;
        y = config.dy;
        theta = config.dtheta;
    }

    void publish() {
        tf::Transform trafo;
        trafo.setRotation(tf::createQuaternionFromYaw(theta));
        trafo.setOrigin(tf::Vector3(x,y,0));

        geometry_msgs::TransformStamped t;
        tf::transformTFToMsg(trafo, t.transform);
        t.header.frame_id = from;
        t.child_frame_id = to;
        t.header.stamp = ros::Time::now();
        tfb.sendTransform(t);
    }

    void spin() {
        ros::WallRate rate(10);
        while(ros::ok()) {
            ros::spinOnce();
            publish();
            rate.sleep();
        }
    }

private:
    tf::TransformBroadcaster tfb;

    dynamic_reconfigure::Server<robot_detection::TfCreatorConfig> server;
    dynamic_reconfigure::Server<robot_detection::TfCreatorConfig>::CallbackType f;

    double x;
    double y;
    double theta;

    std::string from;
    std::string to;
};

int main(int argc, char** argv)
{
    ros::init(argc,argv, "manual_tf_creator");

    ROS_INFO("init");

    ManualTFCreator tfc;

    ROS_INFO("waiting for dynamic reconfigure callback");

    tfc.spin();
}
