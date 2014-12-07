add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function)
add_definitions(-g -rdynamic)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

list(APPEND catkin_LIBRARIES Qt5::Core Qt5::Gui Qt5::Widgets)
list(APPEND catkin_INCLUDE_DIRS ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})

set(CSAPEX_BOOT_PLUGIN_DIR $ENV{HOME}/.csapex/boot)
