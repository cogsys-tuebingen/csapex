#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wall"
#pragma clang diagnostic ignored "-Wpedantic"
#pragma clang diagnostic ignored "-Wdeprecated-declarations"

#elif defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wformat"

#elif defined(_MSC_VER)

#endif

#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
