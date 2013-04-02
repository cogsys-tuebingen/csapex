# - Try to find ViolaJones in Rabot
if (VIOLA_JONES_LIBRARIES)
    set(VIOLA_JONES_FOUND 1)
else ()
    find_path(VIOLA_JONES_INCLUDE_DIR NAMES CascadeDetector.cpp
        PATHS
        $ENV{RABOT}/Vision/ViolaJones
        PATH_SUFFIXES Source
    )
    find_library(VIOLA_JONES_LIBRARY NAMES ViolaJones
        PATHS
        $ENV{RABOT}/Vision/ViolaJones
        PATH_SUFFIXES bin build lib)

    mark_as_advanced(VIOLA_JONES_INCLUDE_DIR)
    set(VIOLA_JONES_INCLUDE_DIRS ${VIOLA_JONES_INCLUDE_DIR} CACHE PATH "The Viola Jones include path.")
    set(VIOLA_JONES_LIBRARIES ${VIOLA_JONES_LIBRARY} CACHE PATH "The Viola Jones libraries.")
    set(VIOLA_JONES_FOUND 1)
endif()

