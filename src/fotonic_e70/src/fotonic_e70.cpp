/// HEADER
#include "fotonic_e70.h"

/// SYSTEM
#include <sstream>

#define LOG(type, a...)\
{ std::stringstream ss; ss << a; \
    g_log_callback(FotonicE70::type, "", ss.str()); }

#define INFO(a...)\
    LOG(LOG_LEVEL_INFO, a);
#define WARN(a...)\
    LOG(LOG_LEVEL_WARN, a);
#define ERROR(a...)\
    LOG(LOG_LEVEL_ERROR, a);

namespace
{
FotonicE70::LogCallback g_log_callback;
void FZLogCallback(char* szMetadata, char* szMessage)
{
  g_log_callback(FotonicE70::LOG_LEVEL_INFO, szMetadata, szMessage);
}
}

FotonicE70::FotonicE70(FrameCallback callback, LogCallback log_callback)
  : frame_callback_(callback)
{
  g_log_callback = log_callback;
}

bool FotonicE70::tick()
{
  size_t iBufSize = sizeof(current_frame_.data);
  FZ_Result iResult = FZ_GetFrame(device_, &current_frame_.header, current_frame_.data, &iBufSize);
  if(iResult != FZ_Success)
  {
    ERROR("FZ_GetFrame failed (code " << iResult << ")");
    return false;
  }
  frame_callback_(current_frame_);
  return true;
}

bool FotonicE70::connectDiscover()
{
  int num_devices = findDevices();
  INFO("Found " << num_devices << " FZ devices");
  
  if(num_devices <= 0)
  {
    return false;
  }
  
  if(!connectToDevice(0))
  {
    ERROR("connection could not be established");
    return false;
  }
  return true;
}

bool FotonicE70::connectStatic(const std::string& static_ip)
{
  if(!connectToDevice(static_ip))
  {
    ERROR("connection to " << static_ip << " could not be established");
    return false;
  }
  
  return true;
}

bool FotonicE70::connect(const std::string& static_ip)
{
  init();
  
  bool has_connection = false;
  if(!static_ip.empty())
  {
    has_connection = connectStatic(static_ip);
    if(!has_connection)
    {
      WARN("static failed, trying discovery");
    }
  }
  
  if(!has_connection)
  {
    has_connection = connectDiscover();
  }
  
  if(!has_connection)
  {
    WARN("discovery failed");
    return false;
  }
  
  uint16_t iDeviceType;
  if(!getDeviceType(&iDeviceType))
  {
    ERROR("cannot get device type");
    return false;
  }
  INFO("DeviceType: " << iDeviceType);
  
  // get and print info strings
  printInfo("FZ_API version", CMD_API_GET_VERSION);
  printInfo("DE version", CMD_DE_GET_VERSION);
  printInfo("CA version", CMD_CA_GET_VERSION);
  printInfo("Product code", CMD_DE_GET_PCODE);
  printInfo("S/N", CMD_DE_GET_UNIT_NO);
  
  // set mode
  if(!setMode(iDeviceType))
  {
    ERROR("cannot set mode");
    return false;
  }
  
  setFrameDataFormat(FZ_FMT_COMPONENT_B | FZ_FMT_COMPONENT_Z | FZ_FMT_COMPONENT_XY | FZ_FMT_PROCESS_INVERTY);
  
  return true;
}

bool FotonicE70::sendCommand(long command, uint16_t value, const std::string& name)
{
  FZ_CmdRespCode_t iRespCode;
  FZ_Result iResult = FZ_IOCtl(device_,
                               command, &value, sizeof(value),
                               &iRespCode, NULL, NULL);
  if(iResult != FZ_Success)
  {
    ERROR("FZ_IOCtl " << name << " failed (code " << iResult << ")");
    return false;
  }
  else
  {
    INFO("FZ_IOCtl " << name << " succeeded (value=" << value << ")");
  }
  
  return true;
}

bool FotonicE70::setShutterSpeed(uint16_t shutterIn10Ms)
{
  return sendCommand(CMD_DE_SET_SHUTTER, shutterIn10Ms, "CMD_DE_SET_SHUTTER");
}

bool FotonicE70::setFrameDataFormat(uint16_t format)
{
  FZ_Result iResult =  FZ_SetFrameDataFmt(device_, -1, -1, -1, -1, format);
  if(iResult != FZ_Success)
  {
    ERROR("FZ_SetFrameDataFmt failed (code " << iResult << ")");
    return false;
  }
  
  return true;
}

bool FotonicE70::setFrameRate(uint16_t fps)
{
  return sendCommand(CMD_DE_SET_FPS, fps, "CMD_DE_SET_FPS");
}

bool FotonicE70::setFrameRateDivisor(uint16_t divisor)
{
  return sendCommand(CMD_DE_SET_FPS_DIVISOR, divisor, "CMD_DE_SET_FPS_DIVISOR");
}

bool FotonicE70::setEdgeFilter(bool enabled)
{
  return sendCommand(CMD_DE_SET_EDGE_FILTER, (uint16_t) enabled, "CMD_DE_SET_EDGE_FILTER");
}

bool FotonicE70::startSensor()
{
  return sendCommand(CMD_DE_SENSOR_START, 0, "CMD_DE_SENSOR_START");
}
bool FotonicE70::stopSensor()
{
  return sendCommand(CMD_DE_SENSOR_STOP, 0, "CMD_DE_SENSOR_STOP");
}

bool FotonicE70::connectToDevice(int index)
{
  FZ_Result iResult = FZ_Open(device_info_[index].szPath, 0, &device_);
  if(iResult != FZ_Success)
  {
    ERROR("FZ_Open (code " << iResult << ")");
    return false;
  }
  
  return true;
}

bool FotonicE70::connectToDevice(const std::string& ip)
{
  FZ_Result iResult = FZ_Open(ip.c_str(), 0, &device_);
  if(iResult != FZ_Success)
  {
    ERROR("FZ_Open (code " << iResult << ")");
    return false;
  }
  
  return true;
}

void FotonicE70::init()
{
  FZ_Init();
  
  int iFlags = FZ_LOG_TO_STDOUT;
  iFlags |= FZ_LOG_ERROR | FZ_LOG_WARN | FZ_LOG_INFO;// | FZ_LOG_TRACE;
  FZ_SetLogging(iFlags, NULL, FZLogCallback);
}

int FotonicE70::findDevices()
{
  int iNumDevices = MAX_DEVICES;
  
  FZ_Result iResult = FZ_EnumDevices2(device_info_, &iNumDevices);
  
  if(iResult == FZ_TOO_MANY_DEVICES)
  {
    iResult = FZ_Success;
  }
  
  if(iResult != FZ_Success)
  {
    ERROR("FZ_EnumDevices2 code " << iResult);
    return -1;
  }
  
  return iNumDevices;
}

bool FotonicE70::getDeviceType(uint16_t* device_type)
{
  FZ_CmdRespCode_t iRespCode;
  int iRespBytes = sizeof(*device_type);
  FZ_Result iResult = FZ_IOCtl(device_,
                               CMD_DE_GET_DEVICE_TYPE, NULL, 0,
                               &iRespCode, device_type, &iRespBytes);
  if(iRespCode != (int)R_CMD_DE_ACK)
  {
    *device_type = FZ_DEVICE_TYPE_JAGUAR;
  }
  if(iResult != FZ_Success)
  {
    ERROR("could not get device type");
    return false;
  }
  
  return true;
}

bool FotonicE70::setMode(uint16_t device_type)
{
  uint16_t mode;
  if(device_type == FZ_DEVICE_TYPE_JAGUAR)
  {
    mode = DE_MODE_BM_TEMPORAL;
    INFO("set mode to DE_MODE_BM_TEMPORAL");
  }
  else if(device_type == FZ_DEVICE_TYPE_PANASONIC)
  {
    mode = DE_MODE_PA_Z;
    INFO("set mode to DE_MODE_PA_Z");
  }
  else if(device_type == FZ_DEVICE_TYPE_PRIMESENSE)
  {
    mode = DE_MODE_640X480_30;
    INFO("set mode to DE_MODE_640X480_30");
  }
  else
  {
    ERROR("Unknown DeviceType (" << device_type << ")");
    return false;
  }
  
  // set the mode
  FZ_CmdRespCode_t iRespCode;
  FZ_Result iResult = FZ_IOCtl(device_,
                               CMD_DE_SET_MODE, &mode, sizeof(mode),
                               &iRespCode, NULL, NULL);
  if(iResult != FZ_Success)
  {
    ERROR("FZ_IOCtl CMD_DE_SET_MODE failed (code " << iResult << ")");
    return false;
  }
  
  return true;
}

bool FotonicE70::getInfo(long id, std::string& target)
{
  char szInfo[128];
  int iRespBytes = 127;
  FZ_CmdRespCode_t iRespCode;
  FZ_Result iResult = FZ_IOCtl(device_,
                               id, NULL, 0,
                               &iRespCode, szInfo, &iRespBytes);
  if(iResult == FZ_Success)
  {
    target = std::string(szInfo);
  }
  
  return iResult == FZ_Success;
}

bool FotonicE70::getInfo(long id, uint16_t& target)
{
  uint16_t szInfo;
  int iRespBytes = 127;
  FZ_CmdRespCode_t iRespCode;
  FZ_Result iResult = FZ_IOCtl(device_,
                               id, NULL, 0,
                               &iRespCode, &szInfo, &iRespBytes);
  if(iResult == FZ_Success)
  {
    target = szInfo;
  }
  
  return iResult == FZ_Success;
}

bool FotonicE70::printInfo(const std::string& name, long id)
{
  std::string info;
  bool result = getInfo(id, info);
  if(!result)
  {
    return false;
  }
  
  INFO(name << ": " << info);
  return true;
}

void FotonicE70::disconnect()
{
  stopSensor();
  FZ_Close(device_);
  FZ_Exit();
}
