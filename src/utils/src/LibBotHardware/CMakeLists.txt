# Utils/LibBotHardware/CMakeLists.txt

include_directories (
  ../../Utils/LibUtil/  ../../Utils/LibRobot/
)

find_package(USB)
find_package(GSL)
find_package(HID)

if(NOT ${USB_FOUND})
    message("not building usb support in LibBotHardware, libusb not found")
else()
    message("found usb: include ${USB_INCLUDE_DIR}")
    include_directories(${USB_INCLUDE_DIR})

    set(USB
        Actuators.cpp
        Avr32UiHandler.cpp
        Basetime.cpp
        CarmenSerial.c
        Compass.cpp
        RobotSensors.cpp
        Sensor.cpp
        Odometry.cpp
        Joystick.cpp
        LaserFx6.cpp
        LogAdapter.cpp
        SerialDevice.cpp
        SpeedController.cpp
        SteerServo.cpp
        Voltage.cpp
        PidSpeedController.cpp
        Ramaxx.cpp
        RamaxxLog.cpp
        Ranger.cpp
        PtzController.cpp
        RtzController.cpp
        WheelEncoder.cpp
        SteerAngleController.cpp
        Avr32Params.cpp
        UsbConn.cpp
    )

endif()

if(NOT ${GSL_FOUND})
    message("not building gsl support in LibBotHardware, gsl not found")
else()
    message("found gsl: include ${GSL_INCLUDE_DIRS}")
    include_directories(${GSL_INCLUDE_DIRS})

    set(GSL
        Adns.cpp
        MouseFlodometry.cpp
    )
endif()

if(NOT ${HID_FOUND})
    message("not building hid support in LibBotHardware, hid not found")
else()
    message("found hid: include ${HID_INCLUDE_DIRS}")
    include_directories(${HID_INCLUDE_DIRS})

    set(HID
        SpacePoint.cpp
    )
endif()



set(REST
    DataListener.cpp
)

add_library (LibBotHardware SHARED
    ${USB} ${GSL} ${HID} ${REST}
)

#target_link_libraries( LibBotHardware  LibUtil LibRobot SDL gsl gslcblas ${USB-1.0_LIBRARIES} hid boost_filesystem)

target_link_libraries( LibBotHardware  LibUtil LibRobot SDL ${GSL_LIBRARIES} ${USB_LIBRARIES} ${HID_LIBRARIES} boost_filesystem)

install (TARGETS LibBotHardware LIBRARY DESTINATION lib)


# INSTALL INCLUDE HEADERS
include(../cmake/install_symlink.cmake)
