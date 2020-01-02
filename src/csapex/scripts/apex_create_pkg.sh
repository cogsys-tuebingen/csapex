#!/bin/bash
dependencies=()


pkg_name=$1
shift;
for item in "$@" ; do
  #process item
  dependencies+=($item)
done
#echo ${dependencies[@]}

mkdir $pkg_name
cd $pkg_name
mkdir src

i=1
n=${#dependencies[@]}
echo "number of deps " $n
dep_list=""
for dep in ${dependencies[@]}; do
	#echo $i
	if [ "$i" == "1" ]; then
		dep_list="$dep""\n"
	elif [ "$i" == "$n" ]; then
		dep_list=$dep_list"    "$dep
	else
 		dep_list=$dep_list"    "$dep"\n"
	fi
        i=$((i+1))
done

printf $dep_list

printf "cmake_minimum_required(VERSION 3.8.2)
project($pkg_name)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    csapex
    $dep_list
    )


catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES 
    CATKIN_DEPENDS
    $dep_list 
    DEPENDS system_lib
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    #include
    \44{catkin_INCLUDE_DIRS}
    )

## Declare a C++ library
add_library(\44{PROJECT_NAME}_node

    )

## Specify libraries to link a library or executable target against
target_link_libraries(\44{PROJECT_NAME}_node
    \44{catkin_LIBRARIES}
    )

install(FILES plugins.xml
    DESTINATION \44{CATKIN_PACKAGE_SHARE_DESTINATION})


install(TARGETS \44{PROJECT_NAME}_node
    ARCHIVE DESTINATION \44{CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION \44{CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION \44{CATKIN_PACKAGE_BIN_DESTINATION})">CMakeLists.txt


printf "<library path=\"lib"$pkg_name"_node\"> \n </library>">plugins.xml

pkg_dep_list=""
j=1
for dep in ${dependencies[@]}; do
	#echo $i
	if [ "$i" == "1" ]; then
		pkg_dep_list="<build_depend>"$dep"</build_depend>\n"
                pkg_dep_list=$pkg_dep_list"  <run_depend>"$dep"</run_depend>\n\n"
	elif [ "$i" == "$n" ]; then
		pkg_dep_list=$pkg_dep_list"  <build_depend>"$dep"</build_depend>\n"
                pkg_dep_list=$pkg_dep_list"  <run_depend>"$dep"</run_depend>\n"
	else
 		pkg_dep_list=$pkg_dep_list"  <build_depend>"$dep"</build_depend>\n"
                pkg_dep_list=$pkg_dep_list"  <run_depend>"$dep"</run_depend>\n\n"
	fi
        i=$((i+1))
done

printf "<?xml version=\"1.0\"?>
<package>
  <name>$pkg_name</name>
  <version>0.0.0</version>
  <description>The $pkg_name package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag --> 
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="\"$USER@todo.todo\"">$USER</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/$pkg_name</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *_depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use run_depend for packages you need at runtime: -->
  <!--   <run_depend>message_runtime</run_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>csapex</build_depend>
  <run_depend>csapex</run_depend>
 
$pkg_dep_list
  <export>
  <csapex plugin=\"\44{prefix}/plugins.xml\" />
  </export>
</package>">package.xml

