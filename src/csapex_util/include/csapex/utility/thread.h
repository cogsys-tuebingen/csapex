#ifndef THREAD_H
#define THREAD_H

/// PROJECT
#include <csapex_util/export.h>

namespace csapex
{
namespace thread
{
CSAPEX_UTILS_EXPORT void set_name(const char* name);
CSAPEX_UTILS_EXPORT const char* get_name();
}  // namespace thread

}  // namespace csapex

#endif  // THREAD_H
