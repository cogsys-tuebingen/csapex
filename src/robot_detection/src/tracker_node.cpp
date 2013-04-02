/// PROJECT
#include <tracker/tracker.h>
#include <ros/tracker_adapter_ros.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv, "robot_tracker");

    Tracker tracker;
    TrackerAdapterRos adapter(tracker);

    return adapter.run(argc, argv);
}

