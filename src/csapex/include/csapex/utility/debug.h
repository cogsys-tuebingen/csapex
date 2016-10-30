#ifndef DEBUG_H
#define DEBUG_H

#define ENABLE_TRACING 0

#if ENABLE_TRACING
#define APEX_DEBUG_TRACE
#define APEX_DEBUG_CERR std::cerr
#else
#define APEX_DEBUG_TRACE if(0)
#define APEX_DEBUG_CERR if(0) std::cerr
#endif

#endif // DEBUG_H
