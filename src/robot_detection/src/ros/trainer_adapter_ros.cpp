/// HEADER
#include "trainer_adapter_ros.h"

/// PROJECT
#include <analyzer/trainer.h>
#include <data/frame_io.h>
#include <data/frame_buffer.h>

/// SYSTEM
#include <angles/angles.h>
#include <background_subtraction/masked_image_handler.h>
#include <background_subtraction/MaskedImage.h>

TrainerAdapterRos::TrainerAdapterRos(Trainer& trainer)
    : TrainerAdapter(trainer), laser_bg(nh), has_imu(false)
{
    img_sub = nh.subscribe<background_subtraction::MaskedImage>
              ("/camera/image_masked", 1, boost::bind(&TrainerAdapterRos::image_callback, this, _1));

    imu_sub = nh.subscribe<sensor_msgs::Imu>
              ("/robot_1/spacepoint/imu", 1, boost::bind(&TrainerAdapterRos::imu_callback, this, _1));
    scan_sub = nh.subscribe<sensor_msgs::LaserScan>
               ("/scan", 1, boost::bind(&TrainerAdapterRos::scan_callback, this, _1));

    nh.param("use_imu", use_imu, true);
    nh.param("use_scan", use_scan, true);
    nh.param("use_tf", use_tf, true);
    nh.param("use_distance", use_distance, true);

    // @TODO make all directories configurable
    nh.param("references_dir", config.ref_dir, config.ref_dir);

    trainer.tick_sig.connect(boost::bind(&TrainerAdapterRos::tick, this, _1));
}

TrainerAdapterRos::~TrainerAdapterRos()
{
}

void TrainerAdapterRos::runHeadless()
{
    spin();
}

void TrainerAdapterRos::tick(double dt)
{
    trainer.tick(dt);
}

void TrainerAdapterRos::imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
    tf::quaternionMsgToTF(msg->orientation, last_imu_orientation);

    // @TODO: why is the imu rotated?
    tf::Quaternion rot = tf::createQuaternionFromRPY(0, M_PI, 0);
    last_imu_orientation = rot * last_imu_orientation;

    has_imu = true;
}

void TrainerAdapterRos::scan_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
    if(use_scan) {
        cv::Mat out;
        laser_bg.process(msg, out);

        FrameBuffer::setImage(1, "laser map", out);

        trainer.signal_frame_analyzed();
        trainer.finalizeDebugImage();
    }
}

void TrainerAdapterRos::image_callback(const background_subtraction::MaskedImageConstPtr& msg)
{
    ros::Time now = msg->header.stamp;
    if(now < last_stamp) {
        WARN("negative time change");
        tfl.clear();
    }

    last_stamp = now;

    bool use_roi = trainer.getState() != Trainer::TRAINING_STOPPED;


    background_subtraction::MaskedImageHandler masked_img_handler;

    cv::Mat image, mask;
    masked_img_handler.import(msg, image, mask);
    Frame::Ptr frame = FrameIO::convert(image, mask, use_roi ? masked_img_handler.findRoi(mask) : cv::Rect());

    if(trainer.getState() != Trainer::TRAINING_STOPPED) {
        get_transformation(msg->header.stamp, frame->orientation, frame->distance);
    }

    trainer.analyze(frame);
}


void TrainerAdapterRos::get_transformation(const ros::Time& time, const Angle& yaw, double& dist)
{
    ROS_WARN_STREAM_THROTTLE(1, "getting transformation");

    // use default:
    tf::Quaternion rot = tf::createQuaternionFromYaw(yaw.toRadians());
    dist = -1;

    // initially use tf if possible
    if(use_tf) {
        std::string from = "/base_link";
        std::string to = "/robot_1/base_link";

        ros::Time stamp = time;

        bool can_transform = tfl.canTransform(from, to, stamp);

        if(!can_transform) {
            stamp = ros::Time(0);
            can_transform = tfl.canTransform(from, to, stamp);
        }

        if(can_transform) {
            ROS_WARN_STREAM_THROTTLE(1, "using TF data");

            tf::StampedTransform transform;
            try {
                tfl.lookupTransform(from, to, stamp, transform);
                rot = transform.getRotation();

                if(use_distance) {
                    dist = transform.getOrigin().length();
                }

            } catch(tf::TransformException& ex) {
                ROS_ERROR("%s",ex.what());;
            }
        } else {
            ROS_WARN_STREAM_THROTTLE(1, "cannot use TF data, transformation not available");
        }
    }

    // refine using imu data, if possible
    if(use_imu && has_imu) {
        ROS_WARN_STREAM_THROTTLE(1, "using IMU rotation");
        rot = last_imu_orientation;
    }

    // refine using scan data, if possible
    if(use_scan && laser_bg.ready()) {
        ROS_WARN_STREAM_THROTTLE(1, "using scan distance");
        dist = laser_bg.dist();
    }


    // apply constant offset
    tf::Quaternion offset = tf::createQuaternionFromYaw(angles::from_degrees(config.angle_offset));
    rot = offset * rot;
}
