MESSAGE(STATUS "Searching for CS::APEX")

SET(CSAPEX_INCLUDE_PATH_DESCRIPTION "top-level directory containing the csapex include directories. E.g /usr/local/include/ or c:\\csapex\\include\\csapex-1.3.2")
SET(CSAPEX_INCLUDE_DIR_MESSAGE "Set the CSAPEX_INCLUDE_DIR cmake cache entry to the ${CSAPEX_INCLUDE_PATH_DESCRIPTION}")
SET(CSAPEX_LIBRARY_PATH_DESCRIPTION "top-level directory containing the csapex libraries.")
SET(CSAPEX_LIBRARY_DIR_MESSAGE "Set the CSAPEX_LIBRARY_DIR cmake cache entry to the ${CSAPEX_LIBRARY_PATH_DESCRIPTION}")


SET(CSAPEX_DIR_SEARCH $ENV{CSAPEX_ROOT})
IF(CSAPEX_DIR_SEARCH)
  FILE(TO_CMAKE_PATH ${CSAPEX_DIR_SEARCH} CSAPEX_DIR_SEARCH)
ENDIF(CSAPEX_DIR_SEARCH)


IF(WIN32)
  SET(CSAPEX_DIR_SEARCH
    ${CSAPEX_DIR_SEARCH}
	C:/Users/buck/ws/apex/install/include
    C:/csapex
    D:/csapex
    "C:/Program Files/csapex"
    "C:/Programme/csapex"
    "D:/Program Files/csapex"
    "D:/Programme/csapex"
  )
ENDIF(WIN32)

SET(SUFFIX_FOR_INCLUDE_PATH
 csapex
)

SET(SUFFIX_FOR_LIBRARY_PATH
 csapex/lib 
)

#
# Look for an installation.
#
FIND_PATH(csapex_INCLUDE_DIR NAMES csapex/csapex_fwd.h PATH_SUFFIXES ${SUFFIX_FOR_INCLUDE_PATH} PATHS
# Look in other places.
  ${CSAPEX_DIR_SEARCH}

# Help the user find it if we cannot.
  DOC "The ${CSAPEX_INCLUDE_PATH_DESCRIPTION}"
)

IF(NOT csapex_INCLUDE_DIR)

# Look for standard unix include paths
  FIND_PATH(csapex_INCLUDE_DIR CSAPEX/CSAPEX.h DOC "The ${CSAPEX_INCLUDE_PATH_DESCRIPTION}")

ENDIF(NOT csapex_INCLUDE_DIR)

# Assume we didn't find it.
SET(CSAPEX_FOUND 0)

# Now try to get the include and library path.
IF(csapex_INCLUDE_DIR)
	SET(csapex_INCLUDE_DIRS
	  ${csapex_INCLUDE_DIR}
	)
	SET(CSAPEX_FOUND 1)


  IF(NOT csapex_LIBRARY_DIR)
    FIND_LIBRARY(CSAPEX_LIB NAMES csapex  PATH_SUFFIXES ${SUFFIX_FOR_LIBRARY_PATH} PATHS

# Look in other places.
      ${csapex_INCLUDE_DIR}/../lib
      ${csapex_INCLUDE_DIR}/../bin
      ${CSAPEX_DIR_SEARCH}

# Help the user find it if we cannot.
      DOC "The ${CSAPEX_LIBRARY_PATH_DESCRIPTION}"
    )
    SET(csapex_LIBRARY_DIR "" CACHE PATH CSAPEX_LIBARARY_PATH_DESCRIPTION)
    GET_FILENAME_COMPONENT(csapex_LIBRARY_DIR ${CSAPEX_LIB} PATH)
    SET(csapex_LIBRARIES "")
    IF(csapex_LIBRARY_DIR AND CSAPEX_LIB)
# Look for the csapex binary path.
      SET(CSAPEX_BINARY_DIR ${csapex_INCLUDE_DIR})
      IF(CSAPEX_BINARY_DIR AND EXISTS "${CSAPEX_BINARY_DIR}/bin")
        SET(CSAPEX_BINARY_DIRS ${CSAPEX_BINARY_DIR}/bin)
      ENDIF(CSAPEX_BINARY_DIR AND EXISTS "${CSAPEX_BINARY_DIR}/bin")
    ENDIF(csapex_LIBRARY_DIR AND CSAPEX_LIB)
    IF(CSAPEX_LIB)
      IF ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
	SET(DBG "d")
      ELSE ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
	SET(DBG "")
      ENDIF ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    ENDIF(CSAPEX_LIB)
    ENDIF(NOT csapex_LIBRARY_DIR)
ENDIF(csapex_INCLUDE_DIR)

IF(NOT CSAPEX_FOUND)
  IF(CSAPEX_FIND_QUIETLY)
    MESSAGE(STATUS "CS::APEX was not found. ${CSAPEX_INCLUDE_DIR_MESSAGE}")
  ELSE(CSAPEX_FIND_QUIETLY)
    IF(CSAPEX_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "CS::APEX was not found. ${CSAPEX_INCLUDE_DIR_MESSAGE}")
    ENDIF(CSAPEX_FIND_REQUIRED)
  ENDIF(CSAPEX_FIND_QUIETLY)
ELSE(NOT CSAPEX_FOUND)
  SET(csapex_LIBRARIES ${CSAPEX_LIB} 
  ${csapex_LIBRARY_DIR}/csapex_util.lib
  ${csapex_LIBRARY_DIR}/csapex_param.lib
  ${csapex_LIBRARY_DIR}/csapex_profiling.lib 
  ${csapex_LIBRARY_DIR}/csapex_param.lib
  ${csapex_LIBRARY_DIR}/csapex_qt.lib
  ${csapex_LIBRARY_DIR}/csapex_command.lib
  )

  list(APPEND CMAKE_MODULE_PATH ${csapex_LIBRARY_DIR}/../CMake)
ENDIF(NOT CSAPEX_FOUND)

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