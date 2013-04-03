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

    message("looking for utils @ ${UTILS_LIBRARY_DIR}")
    file(GLOB_RECURSE UTILS_LIBRARIES_GLOB RELATIVE ${UTILS_LIBRARY_DIR} FOLLOW_SYMLINKS ${UTILS_LIBRARY_DIR}/*.so)

    set(UTILS_LIBRARIES_INCL "-L${UTILS_LIBRARY_DIR}")
    unset(UTILS_LIBRARIES_NAMES)

    foreach(next_ITEM ${UTILS_LIBRARIES_GLOB})
        STRING(REGEX REPLACE "lib([^\\.]+)\\.so" "\\1" lib "${next_ITEM}" )
        if(NOT ${lib} MATCHES "LibRosUtil")
            message(STATUS "> util: ${lib}")
            list(APPEND UTILS_LIBRARIES_INCL "-l${lib}")
            list(APPEND UTILS_LIBRARIES_NAMES ${lib})
        endif()
    endforeach(next_ITEM ${UTILS_LIBRARIES_GLOB})


    file(GLOB_RECURSE UTILS_INCLUDES_GLOB FOLLOW_SYMLINKS RELATIVE ${UTILS_INCLUDE_DIR}/utils ${UTILS_INCLUDE_DIR}/*.h)
    foreach(next_ITEM ${UTILS_INCLUDES_GLOB})
        get_filename_component(path ${next_ITEM} PATH)
        string(FIND ${path} "/" has_slash)
        if(${has_slash} EQUAL -1)
            list(APPEND UTILS_LIBRARY_PATHS ${path})
        endif()
    endforeach(next_ITEM ${UTILS_LIBRARIES_GLOB})

    list(REMOVE_DUPLICATES UTILS_LIBRARY_PATHS)

    mark_as_advanced(UTILS_INCLUDE_DIR)

    list(APPEND UTILS_INCLUDE_DIR ${UTILS_INCLUDE_DIR}/utils)
    list(APPEND UTILS_INCLUDE_DIR ${UTILS_INCLUDE_DIR}/LibRandom/)

    set(UTILS_PATH ${UTILS_DIR} CACHE PATH "The UTILS path.")
    set(UTILS_INCLUDE_DIRS ${UTILS_INCLUDE_DIR} CACHE PATH "The UTILS include path.")
    set(UTILS_LIBRARIES ${UTILS_LIBRARIES_INCL} CACHE PATH "The UTILS libraries.")
    set(UTILS_LIBRARIES_RAW ${UTILS_LIBRARIES_NAMES} CACHE PATH "The UTILS libraries.")

endif()
