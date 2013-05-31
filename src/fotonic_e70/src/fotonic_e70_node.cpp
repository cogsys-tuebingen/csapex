/// COMPONENT
#include "fotonic_e70.h"

/// PROJECT
#include <fotonic_e70/FotonicE70Config.h>

/// SYSTEM
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <signal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace
{
/**
 * this is a fallback, if a signal kills the process -> stop sensor
 */
bool g_stop_requested;
void siginthandler(int param)
{
  g_stop_requested = true;
}
}

/**
 * @brief The FotonicE70Node class is the ROS dependant part of the driver
 */
class FotonicE70Node
{
public:
  /**
  * @brief PointCloud: typedef for a PointCloud with XYZI
  */
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
  
public:
  /**
  * @brief FotonicE70Node Constructor
  */
  FotonicE70Node()
    : driver_(boost::bind(&FotonicE70Node::frameCallback, this, _1),
              boost::bind(&FotonicE70Node::logCallback, this, _1, _2, _3)),
    nh_("~")
  {
    signal(SIGINT, siginthandler);
    signal(SIGTERM, siginthandler);
    signal(SIGKILL, siginthandler);
    
    
    frame_id_ = "fotonic_e70";
    nh_.param("frame_id", frame_id_, frame_id_);
    
    fotonic_e70::FotonicE70Config def_cfg = fotonic_e70::FotonicE70Config::__getDefault__();
    nh_.param("shutter", def_cfg.shutter_speed, def_cfg.shutter_speed);
    nh_.param("fps", def_cfg.fps, def_cfg.fps);
    nh_.param("fps_div", def_cfg.fps_divisor, def_cfg.fps_divisor);
    nh_.param("edge_filter", def_cfg.edge_filter, def_cfg.edge_filter);
    
    std::string static_ip;
    nh_.param("static_ip", static_ip, static_ip);
    
    if(!driver_.connect(static_ip))
    {
      ROS_FATAL_STREAM("could not connect to any sensor.");
      throw;
    }
    
    driver_.startSensor();
    
    point_cloud_publisher_ = nh_.advertise<PointCloud>("cloud", 1, true);

    f = boost::bind(&FotonicE70Node::dynamicReconfigureCallback, this, _1, _2);
    server.setConfigDefault(def_cfg);
    server.setCallback(f);
  }
  
  /**
  * @brief logCallback is used in the ros independant code to send log messages
  * @param level type of log message (INFO, WARN, ERROR)
  * @param meta meta information
  * @param msg log information
  */
  void logCallback(FotonicE70::LOG_LEVEL level, const std::string& meta, const std::string& msg)
  {
    switch(level)
    {
      case FotonicE70::LOG_LEVEL_ERROR:
        ROS_ERROR("[%s] %s", meta.c_str(), msg.c_str());
        break;
      case FotonicE70::LOG_LEVEL_WARN:
        ROS_WARN("[%s] %s", meta.c_str(), msg.c_str());
        break;
      default:
        if(meta.empty())
        {
          ROS_INFO_STREAM(msg);
        }
        else
        {
          ROS_INFO("[%s] %s", meta.c_str(), msg.c_str());
        }
        break;
    }
  }
  
  /**
  * @brief frameCallback is called for each frame of the camera
  * @param frame contains frame data
  */
  void frameCallback(const FotonicE70::Frame& frame)
  {
    unsigned rows = frame.header.nrows;
    unsigned cols = frame.header.ncols;
    
    PointCloud::Ptr msg(new PointCloud);
    msg->header.frame_id = frame_id_;
    msg->width = cols;
    msg->height = rows;
    msg->is_dense = true;
    msg->points.resize(rows * cols, pcl::PointXYZI());
    
    float millimeter2meter = 1e-3;
    float intensity_scale = 0.5 / sizeof(short);
    
    /*
    * Sensor data frame: (y is flipped)    Output frame:
    *        y
    *       |  /                              z
    *       | / z                            |  / x
    * x ____|/                    ->         | /
    *                                   y ___|/
    *
    *
    * Map: x' = z    / 0  1  0 \
    *      y' = x   |  0  0  1 |
    *      z' = y    \ 1  0  0 /
    */
    
    PointCloud::PointType* points = msg->points.data();
    
    unsigned col1 = cols;
    unsigned col2 = 2 * cols;
    unsigned col3 = 3 * cols;
    
    unsigned raw_index = 0;
    for(unsigned row = 0; row < rows; ++row)
    {
      unsigned row_index = row * frame.header.ncols;
      // row format: B0 ... BN, Z0 ... ZN, X0 ... XN, Y0 ... YN
      
      for(unsigned col = 0; col < cols; ++col)
      {
        points[row_index + col].intensity = frame.data[raw_index] * intensity_scale;
        points[row_index + col].x = frame.data[raw_index + col1] * millimeter2meter;
        points[row_index + col].y = frame.data[raw_index + col2] * millimeter2meter;
        points[row_index + col].z = frame.data[raw_index + col3] * millimeter2meter;
        raw_index++;
      }
      
      raw_index += 3 * cols;
    }
    
    point_cloud_publisher_.publish(msg);
  }

  void dynamicReconfigureCallback(fotonic_e70::FotonicE70Config &config, uint32_t level) {
      driver_.setShutterSpeed(config.shutter_speed / 10.0);
      driver_.setFrameRate(config.fps);
      driver_.setFrameRateDivisor(config.fps_divisor);
      driver_.setEdgeFilter(config.edge_filter);
  }
  
  /**
  * @brief run spins the node until it is shut down
  * @return
  */
  int run()
  {
    int loops = 0;
    double ms = 0;
    
    while(ros::ok() && !g_stop_requested)
    {
      // tick blocks, if no new frame is available
      // -> no need for a Rate / WallRate
      ros::WallTime start = ros::WallTime::now();
      if(!driver_.tick())
      {
        ROS_ERROR_STREAM("ticking the driver failed");
        break;
      }
      ros::WallTime stop = ros::WallTime::now();
      
      loops++;
      ms += (stop - start).toNSec() * 1e-6;
      
      if(ms >= 1000.0)
      {
        double fps = loops / ms * 1000;
        ROS_WARN_STREAM_THROTTLE(5, "effective fps " << fps);
        ms -= 1000.0;
        loops = 0;
      }
      
      
      ros::spinOnce();
    }
    
    driver_.disconnect();
    
    return 0;
  }
  
private:
  dynamic_reconfigure::Server<fotonic_e70::FotonicE70Config> server;
  dynamic_reconfigure::Server<fotonic_e70::FotonicE70Config>::CallbackType f;


private:
  FotonicE70 driver_;
  ros::NodeHandle nh_;
  
  std::string frame_id_;
  ros::Publisher point_cloud_publisher_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fotonic_e70_node");
  ROS_INFO_STREAM("starting fotonic e70 node");
  
  FotonicE70Node node;
  return node.run();
}
