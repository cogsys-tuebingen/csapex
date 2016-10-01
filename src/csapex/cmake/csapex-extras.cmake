if(WIN32)
#add_definitions(-Wall)
else()
add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function -Wno-deprecated-register
 -Wno-inconsistent-missing-override -Wno-deprecated-register)
endif()


set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CSAPEX_BOOT_PLUGIN_DIR $ENV{HOME}/.csapex/boot)


## Enforce that we use C++11
if (CMAKE_VERSION VERSION_LESS "3.1")
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  CHECK_CXX_COMPILER_FLAG("-std=gnu++11" COMPILER_SUPPORTS_GNU)
  if(COMPILER_SUPPORTS_CXX11)
     set (CMAKE_CXX_FLAGS "--std=c++11 ${CMAKE_CXX_FLAGS}")
  elseif(COMPILER_SUPPORTS_CXX0X)
     set (CMAKE_CXX_FLAGS "--std=c++0x ${CMAKE_CXX_FLAGS}")
  elseif(COMPILER_SUPPORTS_GNU)
     set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
  else()
     message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
else ()
  set (CMAKE_CXX_STANDARD 11)
endif ()


macro(csapex_package)

        find_package(catkin QUIET)
        if(${catkin_FOUND})
                catkin_package(${ARGN})

        else()
                set(options)
                set(oneValueArgs)
                set(multiValueArgs INCLUDE_DIRS LIBRARIES DEPENDS CATKIN_DEPENDS)
                cmake_parse_arguments(csapex_package "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

                find_package(YAML-CPP REQUIRED)
                find_package(console_bridge REQUIRED)
                find_package(class_loader REQUIRED)
                if(WIN32)
                        add_definitions(/D POCO_NO_AUTOMATIC_LIBS)
                        add_definitions(/D NOMINMAX)
                endif()

                find_package(Boost COMPONENTS program_options filesystem system regex serialization thread REQUIRED)
                find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

                set(INSTALL_DIR ${CMAKE_INSTALL_PREFIX})
                set(CATKIN_PACKAGE_INCLUDE_DESTINATION ${INSTALL_DIR}/include)
                set(CATKIN_PACKAGE_LIB_DESTINATION ${INSTALL_DIR}/lib)
                set(CATKIN_GLOBAL_BIN_DESTINATION ${INSTALL_DIR}/bin)
                set(CATKIN_PACKAGE_SHARE_DESTINATION ${INSTALL_DIR}/share/${PROJECT_NAME})
                set(CSAPEX_MODULE_DESTINATION ${INSTALL_DIR}/CMake)

                set(catkin_INCLUDE_DIRS
                        ${Boost_INCLUDE_DIRS}
                        ${YAML_CPP_INCLUDE_DIR}
                        ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
                        )
                set(catkin_LIBRARIES
                        ${Boost_LIBRARIES}
                        ${YAML_CPP_LIBRARIES}
                        ${class_loader_LIBRARIES}
                        ${console_bridge_LIBRARIES}
                        ${Poco_LIBRARIES}
                        )
                set(csapex_plugin_${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/${csapex_package_INCLUDE_DIRS}" CACHE INTERNAL "build_include")
                set(csapex_plugin_${PROJECT_NAME}_LIBRARIES ${csapex_package_LIBRARIES} CACHE INTERNAL "build_libs")

                if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
                        list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
                        set(CSAPEX_CMAKE_MODULE_PATHS ${CSAPEX_CMAKE_MODULE_PATHS} ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE INTERNAL  "csapex_cmake_dirs")
                endif()
        endif()
endmacro()
