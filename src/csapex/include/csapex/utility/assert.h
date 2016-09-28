#ifndef ASSERT_H
#define ASSERT_H

/// PROJECT
#include <csapex/csapex_util_export.h>

#if WIN32
#define APEX_FUNCTION_SIGNATURE() __FUNCSIG__
#else
#define APEX_FUNCTION_SIGNATURE() __PRETTY_FUNCTION__
#endif

#define apex_assert_msg(assertion,msg)       _apex_assert(static_cast<bool>(assertion),      msg, #assertion, __FILE__, __LINE__, APEX_FUNCTION_SIGNATURE())
#define apex_assert_hard_msg(assertion,msg)  _apex_assert_hard(static_cast<bool>(assertion), msg, #assertion, __FILE__, __LINE__, APEX_FUNCTION_SIGNATURE())
#define apex_assert_soft_msg(assertion,msg)  _apex_assert_soft(static_cast<bool>(assertion), msg, #assertion, __FILE__, __LINE__, APEX_FUNCTION_SIGNATURE())

#define apex_assert(assertion)       apex_assert_msg(assertion,"")
#define apex_assert_hard(assertion)  apex_assert_hard_msg(assertion,"")
#define apex_assert_soft(assertion)  apex_assert_soft_msg(assertion,"")

void CSAPEX_UTILS_EXPORT _apex_assert(bool assertion, const char* msg, const char* code, const char* file, int line, const char* sig);
void CSAPEX_UTILS_EXPORT _apex_assert_hard(bool assertion, const char* msg, const char* code, const char* file, int line, const char* sig);
void CSAPEX_UTILS_EXPORT _apex_assert_soft(bool assertion, const char* msg, const char* code, const char* file, int line, const char* sig);

#endif // ASSERT_H
