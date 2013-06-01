# Utils/Gnuplot/CMakeLists.txt

add_library (Gnuplot
   Gnuplot.cpp
)

#install (TARGETS Gnuplot LIBRARY DESTINATION lib)



# INSTALL INCLUDE HEADERS
include(../cmake/install_symlink.cmake)
