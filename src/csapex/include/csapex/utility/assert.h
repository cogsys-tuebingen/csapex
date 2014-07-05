#ifndef ASSERT_H
#define ASSERT_H

#define apex_assert(assertion)       _apex_assert(assertion,      #assertion, __FILE__, __LINE__)
#define apex_assert_hard(assertion)  _apex_assert_hard(assertion, #assertion, __FILE__, __LINE__)
#define apex_assert_soft(assertion)  _apex_assert_soft(assertion, #assertion, __FILE__, __LINE__)

void _apex_assert(bool assertion, const char* code, const char* file, int line);
void _apex_assert_hard(bool assertion, const char* code, const char* file, int line);
void _apex_assert_soft(bool assertion, const char* code, const char* file, int line);

#endif // ASSERT_H
