#ifndef ASSERT_H
#define ASSERT_H

/// PROJECT
#include <csapex/csapex_util_export.h>

/// SYSTEM
#include <string>

#if WIN32
#define APEX_FUNCTION_SIGNATURE() __FUNCSIG__
#else
#define APEX_FUNCTION_SIGNATURE() __PRETTY_FUNCTION__
#endif

namespace assert
{
inline std::string universal_to_string(const std::string& string)
{
    return string;
}

inline std::string universal_to_string(const char* string)
{
    return std::string(string);
}

template <typename V>
inline std::string universal_to_string(V value)
{
    return std::to_string(value);
}
}

#define apex_assert_msg(assertion,msg)       _apex_assert(static_cast<bool>(assertion),      msg, #assertion, __FILE__, __LINE__, APEX_FUNCTION_SIGNATURE())
#define apex_assert_hard_msg(assertion,msg)  _apex_assert_hard(static_cast<bool>(assertion), msg, #assertion, __FILE__, __LINE__, APEX_FUNCTION_SIGNATURE())
#define apex_assert_soft_msg(assertion,msg)  _apex_assert_soft(static_cast<bool>(assertion), msg, #assertion, __FILE__, __LINE__, APEX_FUNCTION_SIGNATURE())

#define apex_assert_equal(a,b)       apex_assert_msg(((a) == (b)), assert::universal_to_string(a) + " is not equal to " + assert::universal_to_string(b))
#define apex_assert_equal_hard(a,b)  apex_assert_hard_msg(((a) == (b)), assert::universal_to_string(a) + " is not equal to " + assert::universal_to_string(b))
#define apex_assert_equal_soft(a,b)  apex_assert_soft_msg(((a) == (b)), assert::universal_to_string(a) + " is not equal to " + assert::universal_to_string(b))

#define apex_assert_lt(a,b)       apex_assert_msg(((a) < (b)), assert::universal_to_string(a) + " is not smaller than " + assert::universal_to_string(b))
#define apex_assert_lt_hard(a,b)  apex_assert_hard_msg(((a) < (b)), assert::universal_to_string(a) + " is not smaller than " + assert::universal_to_string(b))
#define apex_assert_lt_soft(a,b)  apex_assert_soft_msg(((a) < (b)),  assert::universal_to_string(a) + " is not smaller than " + assert::universal_to_string(b))

#define apex_assert_lte(a,b)       apex_assert_msg(((a) <= (b)), assert::universal_to_string(a) + " is not smaller than or equal to " + assert::universal_to_string(b))
#define apex_assert_lte_hard(a,b)  apex_assert_hard_msg(((a) <= (b)), assert::universal_to_string(a) + " is not smaller than or equal to " + assert::universal_to_string(b))
#define apex_assert_lte_soft(a,b)  apex_assert_soft_msg(((a) <= (b)),  assert::universal_to_string(a) + " is not smaller than or equal to " + assert::universal_to_string(b))

#define apex_assert_gt(a,b)       apex_assert_msg(((a) > (b)), assert::universal_to_string(a) + " is not greater than " + assert::universal_to_string(b))
#define apex_assert_gt_hard(a,b)  apex_assert_hard_msg(((a) > (b)), assert::universal_to_string(a) + " is not greater than " + assert::universal_to_string(b))
#define apex_assert_gt_soft(a,b)  apex_assert_soft_msg(((a) > (b)),  assert::universal_to_string(a) + " is not greater than " + assert::universal_to_string(b))

#define apex_assert_gte(a,b)       apex_assert_msg(((a) >= (b)), assert::universal_to_string(a) + " is not greater than or equal to " + assert::universal_to_string(b))
#define apex_assert_gte_hard(a,b)  apex_assert_hard_msg(((a) >= (b)), assert::universal_to_string(a) + " is not greater than or equal to " + assert::universal_to_string(b))
#define apex_assert_gte_soft(a,b)  apex_assert_soft_msg(((a) >= (b)),  assert::universal_to_string(a) + " is not greater than or equal to " + assert::universal_to_string(b))

#define apex_assert(assertion)       apex_assert_msg(assertion,"")
#define apex_assert_hard(assertion)  apex_assert_hard_msg(assertion,"")
#define apex_assert_soft(assertion)  apex_assert_soft_msg(assertion,"")

void CSAPEX_UTILS_EXPORT _apex_assert(bool assertion, const std::string& msg, const std::string& code, const std::string& file, int line, const std::string& sig);
void CSAPEX_UTILS_EXPORT _apex_assert_hard(bool assertion, const std::string& msg, const std::string& code, const std::string& file, int line, const std::string& sig);
void CSAPEX_UTILS_EXPORT _apex_assert_soft(bool assertion, const std::string& msg, const std::string& code, const std::string& file, int line, const std::string& sig);

#endif // ASSERT_H
