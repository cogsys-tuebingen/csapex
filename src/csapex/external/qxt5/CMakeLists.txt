cmake_minimum_required(VERSION 2.8.11)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(include
    ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
)

add_definitions(-DBUILD_QXT_CORE -DBUILD_QXT_GUI)
add_library(qxt5 SHARED
    src/qxtspanslider.cpp
)

target_link_libraries(qxt5
    Qt5::Core Qt5::Gui Qt5::Widgets
)


#
# INSTALL
#


if(WIN32)
	install(DIRECTORY include/qxt5
			DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
else()
        install(DIRECTORY include/qxt5
			DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
endif()

install(TARGETS qxt5
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
