# - Try to find ViolaJones in Rabot
    find_path(VIOLA_JONES_INCLUDE_DIR NAMES CascadeDetector.cpp
        PATHS
        $ENV{RABOT}/Vision/ViolaJones
        PATH_SUFFIXES Source
    )
    find_library(VIOLA_JONES_LIBRARY NAMES ViolaJones
        PATHS
        $ENV{RABOT}/Vision/ViolaJones
        PATH_SUFFIXES bin build lib)

    if(VIOLA_JONES_LIBRARY AND VIOLA_JONES_INCLUDE_DIR)
        mark_as_advanced(VIOLA_JONES_INCLUDE_DIR)
        set(VIOLA_JONES_INCLUDE_DIRS ${VIOLA_JONES_INCLUDE_DIR} CACHE PATH "The Viola Jones include path.")
        set(VIOLA_JONES_LIBRARIES ${VIOLA_JONES_LIBRARY} CACHE PATH "The Viola Jones libraries.")
        set(VIOLA_JONES_FOUND 1)
    else()
	set(VIOLA_JONES_FOUND 0)
    endif()

