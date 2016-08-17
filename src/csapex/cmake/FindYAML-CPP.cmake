# Locate yaml-cpp
#
# This module defines
#  YAML-CPP_FOUND, if false, do not try to link to yaml-cpp
#  YAML-CPP_LIBRARY, where to find yaml-cpp
#  YAML-CPP_INCLUDE_DIR, where to find yaml.h
#
# By default, the dynamic libraries of yaml-cpp will be found. To find the static ones instead,
# you must set the YAML-CPP_STATIC_LIBRARY variable to TRUE before calling find_package(YAML-CPP ...).
#
# If yaml-cpp is not installed in a standard path, you can use the YAML-CPP_DIR CMake variable
# to tell CMake where yaml-cpp is.

# attempt to find static library first if this is set
if(YAML-CPP_STATIC_LIBRARY)
    set(YAML-CPP_STATIC libyaml-cpp.a)
endif()


# find the yaml-cpp include directory
find_path(YAML-CPP_INCLUDE_DIR yaml-cpp/yaml.h
          PATH_SUFFIXES include
          PATHS
          ~/Library/Frameworks/yaml-cpp/include/
          /Library/Frameworks/yaml-cpp/include/
          /usr/local/include/
          /usr/include/
          /sw/yaml-cpp/         # Fink
          /opt/local/yaml-cpp/  # DarwinPorts
          /opt/csw/yaml-cpp/    # Blastwave
          /opt/yaml-cpp/
          ${YAML-CPP_DIR}/include/)

# find the yaml-cpp library
find_library(YAML-CPP_LIBRARY
             NAMES ${YAML-CPP_STATIC} yaml-cpp
             PATH_SUFFIXES lib64 lib
             PATHS ~/Library/Frameworks
                    /Library/Frameworks
                    /usr/local
                    /usr
                    /sw
                    /opt/local
                    /opt/csw
                    /opt
                    ${YAML-CPP_DIR}/lib)

# handle the QUIETLY and REQUIRED arguments and set YAML-CPP_FOUND to TRUE if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(YAML-CPP DEFAULT_MSG YAML-CPP_INCLUDE_DIR YAML-CPP_LIBRARY)
mark_as_advanced(YAML-CPP_INCLUDE_DIR YAML-CPP_LIBRARY)