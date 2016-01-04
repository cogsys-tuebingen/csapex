#ifndef ASSERT_H
#define ASSERT_H


#define apex_assert_msg(assertion,msg)       _apex_assert(static_cast<bool>(assertion),      msg, #assertion, __FILE__, __LINE__)
#define apex_assert_hard_msg(assertion,msg)  _apex_assert_hard(static_cast<bool>(assertion), msg, #assertion, __FILE__, __LINE__)
#define apex_assert_soft_msg(assertion,msg)  _apex_assert_soft(static_cast<bool>(assertion), msg, #assertion, __FILE__, __LINE__)

#define apex_assert(assertion)       apex_assert_msg(assertion,"")
#define apex_assert_hard(assertion)  apex_assert_hard_msg(assertion,"")
#define apex_assert_soft(assertion)  apex_assert_soft_msg(assertion,"")

void _apex_assert(bool assertion, const char* msg, const char* code, const char* file, int line);
void _apex_assert_hard(bool assertion, const char* msg, const char* code, const char* file, int line);
void _apex_assert_soft(bool assertion, const char* msg, const char* code, const char* file, int line);

#endif // ASSERT_H
