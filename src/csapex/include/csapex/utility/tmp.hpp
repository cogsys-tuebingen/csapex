#ifndef TMP_HPP
#define TMP_HPP

#define HAS_MEM_FUNC(func, name)                                        \
    template<typename T, typename Sign>                                 \
    struct name {                                                       \
    typedef char yes[1];                                            \
    typedef char no [2];                                            \
    template <typename U, U> struct type_check;                     \
    template <typename _1> static yes &chk(type_check<Sign, &_1::func > *); \
    template <typename   > static no  &chk(...);                    \
    static bool const value = sizeof(chk<T>(0)) == sizeof(yes);     \
    }

#define HAS_MEM_FIELD(member, name)                                        \
    template<typename T>                                 \
    struct name {                                                       \
    typedef char yes[1];                                            \
    typedef char no [2];                                            \
    template <typename _1> static yes &chk(typeof(&_1::member)); \
    template <typename   > static no  &chk(...);                    \
    static bool const value = sizeof(chk<T>(0)) == sizeof(yes);     \
    }

#define HAS_MEM_TYPE(member, name)                                        \
    template<typename T>                                 \
    struct name {                                                       \
    typedef char yes[1];                                            \
    typedef char no [2];                                            \
    template <typename _1> static yes &chk(typename _1::member *); \
    template <typename   > static no  &chk(...);                    \
    static bool const value = sizeof(chk<T>(0)) == sizeof(yes);     \
    }

#endif // TMP_HPP

