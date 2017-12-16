#ifndef FUNCTION_TRAITS_HPP
#define FUNCTION_TRAITS_HPP

/// SYSTEM
#include <tuple>

namespace csapex
{

template <typename T>
struct function_traits
    : public function_traits<decltype(&T::operator())>
{};

template <typename ClassType, typename ReturnType, typename... Args>
struct function_traits<ReturnType(ClassType::*)(Args...) const>
{
    enum { arity = sizeof...(Args) };

    using result_type = ReturnType;
    using signature = ReturnType(Args...);

    template <size_t i>
    struct arg
    {
        using type = typename std::tuple_element<i, std::tuple<Args...>>::type;
    };
};


template <typename ReturnType, typename... Args>
struct function_traits<ReturnType(Args...)>
{
    enum { arity = sizeof...(Args) };

    using result_type = ReturnType;
    using signature = ReturnType(Args...);

    template <size_t i>
    struct arg
    {
        using type = typename std::tuple_element<i, std::tuple<Args...>>::type;
    };
};

}

#endif // FUNCTION_TRAITS_HPP
