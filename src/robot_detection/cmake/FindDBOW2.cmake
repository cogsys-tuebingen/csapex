# - Try to find DBoW2 in Rabot
set(DBOW2_FOUND 0)
if (DBOW2_LIBRARIES)
    set(DBOW2_FOUND 1)
else ()
    find_path(DBOW2_INCLUDE_DIR NAMES DBoW2.h
        PATHS
        $ENV{RABOT}/Extlib/DBoW2
    )
    find_library(DBOW2_LIBRARY NAMES DBoW2
        PATHS
        /usr/local/
        $ENV{RABOT}/Extlib/DBoW2
        PATH_SUFFIXES bin build lib)
    find_library(DUTILS_LIBRARY NAMES DUtils
        PATHS
        /usr/local/
        $ENV{RABOT}/Extlib/DBoW2
        PATH_SUFFIXES bin build lib)
    find_library(DVISION_LIBRARY NAMES DVision
        PATHS
        /usr/local/
        $ENV{RABOT}/Extlib/DBoW2
        PATH_SUFFIXES bin build lib)
    find_library(DUTILSCV_LIBRARY NAMES DUtilsCV
        PATHS
        /usr/local/
        $ENV{RABOT}/Extlib/DBoW2
        PATH_SUFFIXES bin build lib)

    mark_as_advanced(DBOW2_INCLUDE_DIR)
    set(INCL ${DBOW2_INCLUDE_DIR} ${DBOW2_INCLUDE_DIR}/DUtils ${DBOW2_INCLUDE_DIR}/DVision ${DBOW2_INCLUDE_DIR}/DUtilsCV)
    set(LIB ${DBOW2_LIBRARY} ${DUTILS_LIBRARY} ${DVISION_LIBRARY} ${DUTILSCV_LIBRARY})
    set(DBOW2_INCLUDE_DIRS ${INCL} CACHE PATH "The DBOW2 include path.")
    set(DBOW2_LIBRARIES ${LIB} CACHE PATH "The DBOW2libraries.")
    if(DBOW2_LIBRARIES)
	    set(DBOW2_FOUND 1)
    else()
	    set(DBOW2_FOUND 0)
    endif()
endif()

