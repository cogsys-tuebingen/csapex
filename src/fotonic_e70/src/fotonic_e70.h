#ifndef FOTONIC_E70_H
#define FOTONIC_E70_H

/// COMPONENT
#include "fz_api.h"

/// SYSTEM
#include <boost/function.hpp>

/**
 * @brief The FotonicE70 class is responsible for talking to the E70 sensor
 */
class FotonicE70
{
public:
  /**
  * @brief The LOG_LEVEL enum is used for log callbacks
  */
  enum LOG_LEVEL
  {
    LOG_LEVEL_INFO, LOG_LEVEL_WARN, LOG_LEVEL_ERROR
  };
  
  /**
   * @brief The Frame struct contains all information necessary to interpret the current frame
   */
  struct Frame
  {
    FZ_FRAME_HEADER header;
    short data[640 * 480 * 4];
  };
  
public:
  typedef boost::function<void(const Frame&)> FrameCallback;
  typedef boost::function<void(LOG_LEVEL, const std::string&, const std::string&)> LogCallback;
  
  /**
   * @brief MAX_DEVICES is the maximumum amount of devices allowed
   */
  static const int MAX_DEVICES = 40;
  
public:
  /**
   * @brief FotonicE70 Constructor
   * @param callback callback for frames
   * @param log_callback callback for logs
   */
  FotonicE70(FrameCallback callback, LogCallback log_callback);
  
  /**
   * @brief connect to the sensor
   * @return <b>false</b> iff a connection couldn't be established
   */
  bool connect(const std::string& static_ip);
  
  /**
   * @brief disconnect closes the connection to the sensor
   */
  void disconnect();
  
  /**
   * @brief tick grabs one frame from the sensor and publishes it
   * @return <b>false</b> iff an error happened
   */
  bool tick();
  
  ///  Setter
  bool setFrameDataFormat(uint16_t format);
  bool setShutterSpeed(uint16_t shutterIn10Ms);
  bool setFrameRate(uint16_t fps);
  bool setFrameRateDivisor(uint16_t divisor);
  bool setEdgeFilter(bool enabled);
  
  bool startSensor();
  bool stopSensor();
  
private:
  void init();
  
  bool connectStatic(const std::string& static_ip);
  bool connectDiscover();
  int findDevices();
  
  
  bool connectToDevice(const std::string& ip);
  bool connectToDevice(int index);
  
  bool getDeviceType(uint16_t* device_type);
  bool setMode(uint16_t device_type);
  
  bool sendCommand(long sendCommand, uint16_t value, const std::string& name);
  
  bool getInfo(long id, std::string& target);
  bool getInfo(long id, uint16_t& target);
  bool printInfo(const std::string& name, long id);
  
private:
  Frame current_frame_;
  
  FZ_DEVICE_INFO device_info_[MAX_DEVICES];
  FZ_Device_Handle_t device_;
  
  FrameCallback frame_callback_;
};

#endif // FOTONIC_E70_H
