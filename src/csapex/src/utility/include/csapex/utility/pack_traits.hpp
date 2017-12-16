#ifndef PACK_TRAITS_HPP
#define PACK_TRAITS_HPP

/// PROJECT
#include <csapex/utility/function_traits.hpp>

/// SYSTEM
#include <type_traits>

namespace csapex
{

namespace detail
{

template<typename Signature,
         int pos, int rest,
         typename ReturnType,
         typename... ArgumentTypes>
struct is_signature_equal_impl
{
    using ArgumentN = typename std::tuple_element<pos, std::tuple<ArgumentTypes...>>::type;
    using SignatureArgumentN = typename function_traits<Signature>::template arg<pos>::type;
    enum {
        value = std::is_same<ArgumentN, SignatureArgumentN>::value &&
                is_signature_equal_impl<Signature, pos+1, rest-1, ReturnType, ArgumentTypes...>::value
    };
};


template<typename Signature,
         int pos,
         typename ReturnType,
         typename... ArgumentTypes>
struct is_signature_equal_impl<Signature, pos, 0, ReturnType, ArgumentTypes...>
{
    enum {
        value = std::is_same<ReturnType, typename function_traits<Signature>::result_type>::value
    };
};

}



template<typename Signature,
         typename ReturnType,
         typename... ArgumentTypes>
struct is_signature_equal
{
    enum {
        arity = function_traits<Signature>::arity,
        value = (sizeof...(ArgumentTypes) == arity) &&
                detail::is_signature_equal_impl<Signature, 0, sizeof...(ArgumentTypes), ReturnType, ArgumentTypes...>::value
    };
};


}

#endif // PACK_TRAITS_HPP
