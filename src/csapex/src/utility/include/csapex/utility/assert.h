#ifndef ASSERT_H
#define ASSERT_H

/// PROJECT
#include <csapex/csapex_util_export.h>
#include <csapex/utility/type.h>
#include <csapex/utility/string.hpp>

/// SYSTEM
#include <string>
#include <memory>

#if WIN32
#define APEX_FUNCTION_SIGNATURE() __FUNCSIG__
#else
#define APEX_FUNCTION_SIGNATURE() __PRETTY_FUNCTION__
#endif

#define apex_assert_msg(assertion,msg)       _apex_assert(static_cast<bool>(assertion),      msg, #assertion, __FILE__, __LINE__, APEX_FUNCTION_SIGNATURE())
#define apex_assert_hard_msg(assertion,msg)  _apex_assert_hard(static_cast<bool>(assertion), msg, #assertion, __FILE__, __LINE__, APEX_FUNCTION_SIGNATURE())
#define apex_assert_soft_msg(assertion,msg)  _apex_assert_soft(static_cast<bool>(assertion), msg, #assertion, __FILE__, __LINE__, APEX_FUNCTION_SIGNATURE())

#define apex_assert_eq(a,b)       apex_assert_msg(((a) == (b)), universal_to_string(a) + " is not equal to " + universal_to_string(b))
#define apex_assert_eq_hard(a,b)  apex_assert_hard_msg(((a) == (b)), universal_to_string(a) + " is not equal to " + universal_to_string(b))
#define apex_assert_eq_soft(a,b)  apex_assert_soft_msg(((a) == (b)), universal_to_string(a) + " is not equal to " + universal_to_string(b))

#define apex_assert_neq(a,b)       apex_assert_msg(((a) != (b)), universal_to_string(a) + " is equal to " + universal_to_string(b))
#define apex_assert_neq_hard(a,b)  apex_assert_hard_msg(((a) != (b)), universal_to_string(a) + " is equal to " + universal_to_string(b))
#define apex_assert_neq_soft(a,b)  apex_assert_soft_msg(((a) != (b)), universal_to_string(a) + " is equal to " + universal_to_string(b))

#define apex_assert_equal(a,b)       apex_assert_eq(a,b)
#define apex_assert_equal_hard(a,b)  apex_assert_eq_hard(a,b)
#define apex_assert_equal_soft(a,b)  apex_assert_eq_soft(a,b)

#define apex_assert_lt(a,b)       apex_assert_msg(((a) < (b)), universal_to_string(a) + " is not smaller than " + universal_to_string(b))
#define apex_assert_lt_hard(a,b)  apex_assert_hard_msg(((a) < (b)), universal_to_string(a) + " is not smaller than " + universal_to_string(b))
#define apex_assert_lt_soft(a,b)  apex_assert_soft_msg(((a) < (b)),  universal_to_string(a) + " is not smaller than " + universal_to_string(b))

#define apex_assert_lte(a,b)       apex_assert_msg(((a) <= (b)), universal_to_string(a) + " is not smaller than or equal to " + universal_to_string(b))
#define apex_assert_lte_hard(a,b)  apex_assert_hard_msg(((a) <= (b)), universal_to_string(a) + " is not smaller than or equal to " + universal_to_string(b))
#define apex_assert_lte_soft(a,b)  apex_assert_soft_msg(((a) <= (b)),  universal_to_string(a) + " is not smaller than or equal to " + universal_to_string(b))

#define apex_assert_gt(a,b)       apex_assert_msg(((a) > (b)), universal_to_string(a) + " is not greater than " + universal_to_string(b))
#define apex_assert_gt_hard(a,b)  apex_assert_hard_msg(((a) > (b)), universal_to_string(a) + " is not greater than " + universal_to_string(b))
#define apex_assert_gt_soft(a,b)  apex_assert_soft_msg(((a) > (b)),  universal_to_string(a) + " is not greater than " + universal_to_string(b))

#define apex_assert_gte(a,b)       apex_assert_msg(((a) >= (b)), universal_to_string(a) + " is not greater than or equal to " + universal_to_string(b))
#define apex_assert_gte_hard(a,b)  apex_assert_hard_msg(((a) >= (b)), universal_to_string(a) + " is not greater than or equal to " + universal_to_string(b))
#define apex_assert_gte_soft(a,b)  apex_assert_soft_msg(((a) >= (b)),  universal_to_string(a) + " is not greater than or equal to " + universal_to_string(b))

#define apex_assert(assertion)       apex_assert_msg(assertion,"")
#define apex_assert_hard(assertion)  apex_assert_hard_msg(assertion,"")
#define apex_assert_soft(assertion)  apex_assert_soft_msg(assertion,"")


#define apex_fail(msg) _apex_fail(msg, "", __FILE__, __LINE__, APEX_FUNCTION_SIGNATURE())

void CSAPEX_UTILS_EXPORT _apex_fail(const std::string& msg, const std::string& code, const std::string& file, int line, const std::string& sig);
void CSAPEX_UTILS_EXPORT _apex_assert(bool assertion, const std::string& msg, const std::string& code, const std::string& file, int line, const std::string& sig);
void CSAPEX_UTILS_EXPORT _apex_assert_hard(bool assertion, const std::string& msg, const std::string& code, const std::string& file, int line, const std::string& sig);
void CSAPEX_UTILS_EXPORT _apex_assert_soft(bool assertion, const std::string& msg, const std::string& code, const std::string& file, int line, const std::string& sig);

#endif // ASSERT_H
