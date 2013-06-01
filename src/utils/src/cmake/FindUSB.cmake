# - Try to find USB
# Once done this will define
#
#  USB_FOUND - system has USB
#  USB_INCLUDE_DIRS - the USB include directory
#  USB_LIBRARIES - Link these to use USB
#  USB_DEFINITIONS - Compiler switches required for using USB

if (USB_LIBRARIES AND USB_INCLUDE_DIRS)
  # in cache already
  set(USB_FOUND TRUE)
else (USB_LIBRARIES AND USB_INCLUDE_DIRS)
  set(USB_FOUND FALSE)

  # use pkg-config to get the directories and then use these values
  # in the FIND_PATH() and FIND_LIBRARY() calls
  if (${CMAKE_MAJOR_VERSION} EQUAL 2 AND ${CMAKE_MINOR_VERSION} EQUAL 4)
    include(UsePkgConfig)
    pkgconfig(libUSB _USB_INCLUDEDIR _USB_LIBDIR _USB_LDFLAGS _USB_CFLAGS)
  else (${CMAKE_MAJOR_VERSION} EQUAL 2 AND ${CMAKE_MINOR_VERSION} EQUAL 4)
    find_package(PkgConfig)
    if (PKG_CONFIG_FOUND)
      pkg_check_modules(_USB libUSB)
    endif (PKG_CONFIG_FOUND)
  endif (${CMAKE_MAJOR_VERSION} EQUAL 2 AND ${CMAKE_MINOR_VERSION} EQUAL 4)

  find_path(USB_INCLUDE_DIR
    NAMES
      usb.h
    PATHS
      ${_USB_INCLUDEDIR}
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
    PATH_SUFFIXES
      libUSB
  )
  mark_as_advanced(USB_INCLUDE_DIR)

  find_library(USB_LIBRARY
    NAMES
      usb
    PATHS
      ${_USB_LIBDIR}
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )
  mark_as_advanced(USB_LIBRARY)

  if (USB_LIBRARY)
    set(USB_FOUND TRUE)
    mark_as_advanced(USB_FOUND)
  endif (USB_LIBRARY)

  set(USB_INCLUDE_DIRS
    ${USB_INCLUDE_DIR}
  )

  if (USB_FOUND)
    set(USB_LIBRARIES
      ${USB_LIBRARIES}
      ${USB_LIBRARY}
    )
  endif (USB_FOUND)

  if (USB_INCLUDE_DIRS AND USB_LIBRARIES)
     set(USB_FOUND TRUE)
  endif (USB_INCLUDE_DIRS AND USB_LIBRARIES)

  if (USB_FOUND)
    if (NOT USB_FIND_QUIETLY)
      message(STATUS "Found USB: ${USB_LIBRARIES}")
    endif (NOT USB_FIND_QUIETLY)
  else (USB_FOUND)
    if (USB_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find USB")
    endif (USB_FIND_REQUIRED)
  endif (USB_FOUND)

  # show the USB_INCLUDE_DIRS and USB_LIBRARIES variables only in the advanced view
  mark_as_advanced(USB_INCLUDE_DIRS USB_LIBRARIES)

endif (USB_LIBRARIES AND USB_INCLUDE_DIRS)

