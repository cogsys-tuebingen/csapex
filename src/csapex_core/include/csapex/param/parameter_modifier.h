#ifndef PARAMETER_MODIFIER_H
#define PARAMETER_MODIFIER_H

/// COMPONENT
#include <csapex/param/param_fwd.h>
#include <csapex_param_export.h>

/// SYSTEM
#include <string>
#include <type_traits>

namespace csapex
{
namespace param
{
template <typename T, class Enable = void>
class ParameterModifier
{
public:
    static void set(const ParameterPtr& p, T value)
    {
        static_assert(std::is_same<T, int>::value || std::is_same<T, bool>::value || !std::is_integral<T>::value, "invalid call to ParameterModifier with integer type");
        p->set<T>(value);
    }
    static bool setSilent(const ParameterPtr& p, T value)
    {
        return p->setSilent<T>(value);
    }

    static T get(const ParameterConstPtr& p)
    {
        return p->as<T>();
    }
};

template <typename T>
class CSAPEX_PARAM_EXPORT ParameterModifier<T, typename std::enable_if<std::is_floating_point<T>::value>::type>
{
public:
    static void set(const ParameterPtr& p, T value)
    {
        p->set<double>(static_cast<double>(value));
    }
    static bool setSilent(const ParameterPtr& p, T value)
    {
        return p->setSilent<double>(static_cast<double>(value));
    }

    static T get(const ParameterConstPtr& p)
    {
        return static_cast<T>(p->as<double>());
    }
};

template <typename T>
class CSAPEX_PARAM_EXPORT ParameterModifier<T, typename std::enable_if<std::is_integral<T>::value && !std::is_same<T, bool>::value>::type>
{
public:
    static void set(const ParameterPtr& p, T value)
    {
        p->set<int>(static_cast<int>(value));
    }
    static bool setSilent(const ParameterPtr& p, T value)
    {
        return p->setSilent<int>(static_cast<int>(value));
    }

    static T get(const ParameterConstPtr& p)
    {
        return static_cast<T>(p->as<int>());
    }
};

}  // namespace param
}  // namespace csapex

#endif  // PARAMETER_MODIFIER_H
