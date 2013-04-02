/// SYSTEM
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>

class Trafo
{
public:
    Trafo() {}

    Trafo(const std::string& from, const std::string& to) {
        this->from = from;
        this->to = to;
    }

    bool equals(const geometry_msgs::TransformStamped& t) {
        return t.header.frame_id == from && t.child_frame_id == to;
    }

    std::string from;
    std::string to;
};

class CallbackProvider
{
public:
    template<typename T>
    T operator()(const boost::shared_ptr<T const> &msg, ros::Publisher& pub, const std::string& tf) {
        T p = *msg;
        if(!tf.empty()) {
            p.header.frame_id = tf;
        }
        pub.publish(p);
        return p;
    }
};

class Rerouter
{

public:
    Rerouter()
        : nh_("~"), has_target_transformation_(false), target_transformation_fixed_(false), is_master_(false), is_slave_(false)
    {}

    void init() {
        std::string robot_0 = "/robot_0";
        std::string robot_1 = "/robot_1";
        std::string robot_0_ns = "";
        std::string robot_1_ns = "";

        std::string target_frame = robot_0 + "/map";
        std::string laser_frame = "/base_laser_link";

        nh_.param("robot_0", robot_0, robot_0);
        nh_.param("robot_1", robot_1, robot_1);

        nh_.param("robot_0_ns", robot_0_ns, robot_0_ns);
        nh_.param("robot_1_ns", robot_1_ns, robot_1_ns);

        nh_.param("target_frame", target_frame, target_frame);
        nh_.param("laser_frame", laser_frame, laser_frame);

        nh_.param("master", is_master_, false);
        nh_.param("slave", is_slave_, false);

        trafo_sync_ = Trafo("/sync/base_link", "/sync/map");
        trafo_real_ = Trafo(robot_1 + "/base_link", target_frame);
        trafo_target_ = Trafo("/map", robot_1 + "/map");

        /*
         * Subscription for commands
         */
        subs_["sys"] = nh_.subscribe<std_msgs::String>
                       ("/syscommand", 5, boost::bind(&Rerouter::syscommand, this, _1));


        /*
         * Mappings Robot 1 <-> Sync
         */
        if(is_master()) {
            // map second robot's scan to /sync
            map_topic<sensor_msgs::LaserScan>(robot_1 + "/scan", "/sync/scan", "/sync" + laser_frame, 20, 20);

            // map first robot's map to /sync
            map_topic<nav_msgs::OccupancyGrid>(robot_0_ns + "/map", "/sync/map", "/sync/map", 1, 1);

            // map sync's particle cloud back to first robot's frame
            map_topic<geometry_msgs::PoseArray>("/particlecloud", "/particlecloud_final", target_frame, 1, 1);

            // copy the second robot's tf over to /sync
            map_tf(robot_1, "/sync");
        }

        /*
         * Mappings Robot 1 <-> Robot 0 for multimaster
         *
         * Robot 0 needs Robot 1's scan, map and pose
         */
        if(is_slave()) {
            map_topic<sensor_msgs::LaserScan>(robot_1_ns + "/scan", robot_1 + "/scan", robot_1 + laser_frame, 20, 20);
            map_topic<nav_msgs::OccupancyGrid>(robot_1_ns + "/map", robot_1 + "/map", robot_1 + "/map", 1, 1);
            map_topic<geometry_msgs::PoseStamped>(robot_1_ns + "/slam_out_pose", robot_1 + "/pose", robot_1 + "/map", 20, 20);
        }
    }

    void syscommand(const std_msgs::StringConstPtr& msg) {
        if(msg->data == "fix") {
            if(!has_target_transformation_) {
                ROS_ERROR_STREAM("cannot fixate the target transformation, none found yet...");
                return;
            }
            ROS_WARN_STREAM("fixating the current transformation");
            target_transformation_fixed_ = true;
        } else if(msg->data == "unfix") {
            target_transformation_fixed_ = false;
        } else {
            ROS_ERROR_STREAM("unknown syscommand: " << msg->data);
        }
    }

    bool replace(std::string& str, const std::string& from, const std::string& to) {
        size_t start_pos = str.find(from);
        if(start_pos == std::string::npos)
            return false;
        str.replace(start_pos, from.length(), to);
        return true;
    }

    bool begins_with(const std::string& str, const std::string& begin) {
        return str.substr(0, begin.size()) == begin;
    }

    void map_tf(const tf::tfMessageConstPtr& msg, const std::string& from, const std::string& to) {
        std::string ignore_this = from + "/map";
        int n = msg->transforms.size();
        for(int i=0; i < n; ++i) {
            geometry_msgs::TransformStamped trafo = msg->transforms[i];
            std::string& frame = trafo.header.frame_id;

            // check if this frame should be transformed
            if(begins_with(frame, to) || begins_with(frame, ignore_this) || !begins_with(frame, from)) {
                continue;
            }
            // check if this frame is the target frame -> no transormation!
            if(trafo_sync_.equals(trafo) || trafo_real_.equals(trafo)) {
                continue;
            }

            replace(trafo.child_frame_id, from, to);
            replace(trafo.header.frame_id, from, to);

            tfb_.sendTransform(trafo);
        }
    }

    void map_tf(const std::string& from, const std::string& to) {
        subs_["tf:" + from] = nh_.subscribe<tf::tfMessage>
                              ("/tf", 10, boost::bind(&Rerouter::map_tf, this, _1, from, to));
    }

    void get_target_transformation() {
        if(!tfl_.canTransform(trafo_target_.from, trafo_target_.to, ros::Time(0))) {
            ROS_WARN_STREAM_THROTTLE(1, "cannot get the target transformation! (" << trafo_target_.from << " to " << trafo_target_.to << ")");
            return;
        }

        try {
            tfl_.lookupTransform(trafo_target_.from, trafo_target_.to, ros::Time(0), target_transformation_);

        } catch(tf::TransformException& ex) {
            ROS_ERROR("%s",ex.what());;
            return;
        }

        has_target_transformation_ = true;
    }

    void publish_tf() {
        geometry_msgs::TransformStamped trafo;

        get_target_transformation();

        if(target_transformation_fixed_) {
            tf::transformStampedTFToMsg(target_transformation_, trafo);

            trafo.header.frame_id = trafo_target_.from;
            trafo.child_frame_id = trafo_target_.to;
        } else {

            tf::StampedTransform current_transformation;

            if(!tfl_.canTransform(trafo_sync_.from, trafo_sync_.to, ros::Time(0))) {
                ROS_WARN_STREAM_THROTTLE(1, "cannot get the sync transformation! (" << trafo_sync_.from << " to " << trafo_sync_.to << ")");
                return;
            }

            try {
                tfl_.lookupTransform(trafo_sync_.from, trafo_sync_.to, ros::Time(0), current_transformation);

            } catch(tf::TransformException& ex) {
                ROS_ERROR("%s",ex.what());;
                return;
            }

            tf::transformStampedTFToMsg(current_transformation, trafo);

            trafo.header.frame_id = trafo_real_.from;
            trafo.child_frame_id = trafo_real_.to;
        }

        trafo.header.stamp = ros::Time::now();
        tfb_.sendTransform(trafo);
    }

    template <typename T>
    void map_topic(const std::string& from, const std::string& to, const std::string& tf = "", int queue_in = 1, int queue_out = 1) {
        if(from == to) {
            ROS_WARN_STREAM("dropped mapping from " << from << " to " << to);
            return;
        }

        ROS_INFO_STREAM("mapping from " << from << " to " << to);

        ROS_ERROR_STREAM_COND(pubs_.count(from) > 0, "multiple mappings from " << from);
        ROS_ERROR_STREAM_COND(subs_.count(to) > 0, "multiple mappings to " << to);

        pubs_[from] = nh_.advertise<T>
                      (to, queue_out, true);

        subs_[from] = nh_.subscribe<T>
                      (from, queue_in, boost::bind<T>(CallbackProvider(), _1, pubs_[from], tf));
    }

    bool is_master() {
        return is_master_;
    }

    bool is_slave() {
        return is_slave_;
    }

private:
    ros::NodeHandle nh_;

    std::map<std::string, ros::Publisher> pubs_;
    std::map<std::string, ros::Subscriber> subs_;

    tf::TransformBroadcaster tfb_;
    tf::TransformListener tfl_;

    bool has_target_transformation_;
    bool target_transformation_fixed_;
    tf::StampedTransform target_transformation_;

    Trafo trafo_sync_;
    Trafo trafo_real_;
    Trafo trafo_target_;

    bool is_master_;
    bool is_slave_;
};

int main(int argc, char** argv)
{
    ros::init(argc,argv, "rerouter");

    Rerouter rerouter;

    rerouter.init();

    ros::WallRate rate(20);

    while(ros::ok()) {
        ros::spinOnce();
        rerouter.publish_tf();
        rate.sleep();
    }
}
