#ifndef TYPE_TRAITS_HPP
#define TYPE_TRAITS_HPP

/// SYSTEM
#include <type_traits>

namespace csapex
{
namespace detail
{
template <typename T>
struct is_complete_helper
{
    template <typename U>
    static auto test(U*) -> std::integral_constant<bool, sizeof(U) == sizeof(U)>;
    static auto test(...) -> std::false_type;
    using type = decltype(test(static_cast<T*>(0)));
};

struct Dummy
{
};
template <typename T, typename Arg>
Dummy operator<<(const T&, const Arg&);
template <typename T, typename Arg>
struct is_left_shift_operator_defined
{
    template <typename U, typename V>
    static auto test(U*, V*) -> std::integral_constant<bool, !std::is_same<decltype(std::declval<T>() << std::declval<Arg>()), Dummy>::value>;
    static auto test(...) -> std::false_type;
    using type = decltype(test(static_cast<T*>(0), static_cast<Arg*>(0)));
};
template <typename T, typename Arg>
Dummy operator>>(const T&, const Arg&);
template <typename T, typename Arg>
struct is_right_shift_operator_defined
{
    template <typename U, typename V>
    static auto test(U*, V*) -> std::integral_constant<bool, !std::is_same<decltype(std::declval<T>() >> std::declval<Arg>()), Dummy>::value>;
    static auto test(...) -> std::false_type;
    using type = decltype(test(static_cast<T*>(0), static_cast<Arg*>(0)));
};

}  // namespace detail

/// Check if a type is complete
template <typename T>
struct is_complete : detail::is_complete_helper<T>::type
{
};
template <typename T>
inline constexpr bool is_complete_v = is_complete<T>::value;

/// Check if the expression T << Args is well defined
template <typename T, typename Args>
struct is_left_shift_operator_defined : detail::is_left_shift_operator_defined<T, Args>::type
{
};
template <typename T, typename Args>
struct is_right_shift_operator_defined : detail::is_right_shift_operator_defined<T, Args>::type
{
};

template <typename T, typename Arg>
inline constexpr bool is_left_shift_operator_defined_v = is_left_shift_operator_defined<T, Arg>::value;
template <typename T, typename Arg>
inline constexpr bool is_right_shift_operator_defined_v = is_right_shift_operator_defined<T, Arg>::value;

}  // namespace csapex

#endif  // TYPE_TRAITS_HPP
