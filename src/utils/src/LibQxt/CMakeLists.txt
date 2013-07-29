
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

set(MOC_HDRS
    qxtspanslider.h
    qxtspanslider_p.h
)

qt4_wrap_cpp(MOC_SRCS ${MOC_HDRS})

include_directories(include
  .
)

add_library(LibQxt
    qxtspanslider.cpp
    ${MOC_SRCS}
)

target_link_libraries(LibQxt
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES})

# INSTALL INCLUDE HEADERS
include(../cmake/install_symlink.cmake)

