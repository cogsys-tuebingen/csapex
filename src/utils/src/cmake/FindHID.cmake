if (HID_LIBRARIES AND HID_INCLUDE_DIRS)
  # in cache already
  set(HID_FOUND TRUE)
else (HID_LIBRARIES AND HID_INCLUDE_DIRS)
  set(HID_FOUND FALSE)

  find_path(HID_INCLUDE_DIR
    NAMES
      hid.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )
  mark_as_advanced(HID_INCLUDE_DIR)

  find_library(HID_LIBRARY
    NAMES
      hid
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )
  mark_as_advanced(HID_LIBRARY)

  if (HID_LIBRARY)
    set(HID_FOUND TRUE)
    mark_as_advanced(HID_FOUND)
  endif (HID_LIBRARY)

  set(HID_INCLUDE_DIRS
    ${HID_INCLUDE_DIR}
  )

  if (HID_FOUND)
    set(HID_LIBRARIES
      ${HID_LIBRARIES}
      ${HID_LIBRARY}
    )
  endif (HID_FOUND)

  if (HID_INCLUDE_DIRS AND HID_LIBRARIES)
     set(HID_FOUND TRUE)
  endif (HID_INCLUDE_DIRS AND HID_LIBRARIES)

  if (HID_FOUND)
    if (NOT HID_FIND_QUIETLY)
      message(STATUS "Found HID: ${HID_LIBRARIES}")
    endif (NOT HID_FIND_QUIETLY)
  else (HID_FOUND)
    if (HID_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find HID")
    endif (HID_FIND_REQUIRED)
  endif (HID_FOUND)

  # show the HID_INCLUDE_DIRS and HID_LIBRARIES variables only in the advanced view
  mark_as_advanced(HID_INCLUDE_DIRS HID_LIBRARIES)

endif (HID_LIBRARIES AND HID_INCLUDE_DIRS)

