#ifndef GLOBAL_H
#define GLOBAL_H

#define DEBUG 1
#define USE_ROS_CONSOLE 0

#if DEBUG
#define DEBUG_CODE(code) {code}
#else
#define DEBUG_CODE(code) {}
#endif


#if USE_ROS_CONSOLE
#include <rosconsole/macros_generated.h>

#define INFO(args) ROS_INFO_STREAM(args)
#define WARN(args) ROS_WARN_STREAM(args)
#define ERROR(args) ROS_ERROR_STREAM(args)
#define FATAL(args) ROS_FATAL_STREAM(args)

#else
#include "logger.h"

#define STREAM(out, args) {(out << args); out.endl();}
#define INFO_ Logger::INFO
#define INFO(args) STREAM( INFO_, args)
#define WARN_ Logger::WARN
#define WARN(args) STREAM( WARN_, args)
#define ERROR_ Logger::ERROR
#define ERROR(args) STREAM( ERROR_, args)
#define FATAL_ Logger::FATAL
#define FATAL(args) STREAM(FATAL_, args)
#endif

#endif // GLOBAL_H
