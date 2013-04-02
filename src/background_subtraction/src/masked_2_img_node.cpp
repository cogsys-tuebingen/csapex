/// PROJECT
#include <background_subtraction/masked_image_handler.h>

/// SYSTEM
#include <ros/ros.h>

using namespace background_subtraction;

MaskedImageHandler handler;

ros::Publisher pub;

std::string read_topic = "/camera/image_masked";
std::string write_topic = "/camera/image_raw";

void callback(const background_subtraction::MaskedImageConstPtr& msg)
{
    cv::Mat mask;
    cv_bridge::CvImage cv;
    handler.import(msg, cv.image, mask);

    cv.header = msg->header;
    cv.encoding = sensor_msgs::image_encodings::BGR8;

    pub.publish(cv.toImageMsg());

    ROS_INFO_STREAM("published from " << read_topic << " to "  << write_topic);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "masked2img");

    ros::NodeHandle nh("~");

    ros::Subscriber s = nh.subscribe<background_subtraction::MaskedImage>
                        (read_topic, 1, callback);

    pub = nh.advertise<sensor_msgs::Image>
          (write_topic, 1, true);

    ros::spin();
}
