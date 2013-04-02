# - Try to find Utils in Rabot
find_path(UTILS_DIR NAMES include/utils/LibUtil/MathHelper.h
    PATHS
    $ENV{RABOT}/ros/utils/
    ../../../ros/utils/
)

if(UTILS_DIR)
    set(UTILS_FOUND TRUE)
else()
    set(UTILS_FOUND FALSE)

    if(Utils_FIND_REQUIRED)
        message(ERROR ${UTILS_DIR})
        message(FATAL_ERROR "Please install the rabot utils package, it could not be found")
    endif(Utils_FIND_REQUIRED)

endif()

if(UTILS_FOUND)
    set(UTILS_INCLUDE_DIR "${UTILS_DIR}/include")
    set(UTILS_LIBRARY_DIR "${UTILS_DIR}/lib")

    file(GLOB_RECURSE UTILS_LIBRARIES_GLOB RELATIVE ${UTILS_LIBRARY_DIR} FOLLOW_SYMLINKS ${UTILS_LIBRARY_DIR}/*.so)

    set(UTILS_LIBRARIES_INCL "-L${UTILS_LIBRARY_DIR}")

    foreach(next_ITEM ${UTILS_LIBRARIES_GLOB})
        STRING(REGEX REPLACE "lib([^\\.]+)\\.so" "\\1" lib "${next_ITEM}" )
        list(APPEND UTILS_LIBRARIES_INCL "-l${lib}")
    endforeach(next_ITEM ${UTILS_LIBRARIES_GLOB})

    mark_as_advanced(UTILS_INCLUDE_DIR)

    list(APPEND UTILS_INCLUDE_DIR ${UTILS_INCLUDE_DIR}/utils)
    list(APPEND UTILS_INCLUDE_DIR ${UTILS_INCLUDE_DIR}/LibRandom/)

    set(UTILS_INCLUDE_DIRS ${UTILS_INCLUDE_DIR} CACHE PATH "The UTILS include path.")
    set(UTILS_LIBRARIES ${UTILS_LIBRARIES_INCL} CACHE PATH "The UTILS libraries.")

endif()
