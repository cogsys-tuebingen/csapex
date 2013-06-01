# src/CMakeLists.txt

include_directories (
    LibUtil
    LibGeneric
    LibRobot
    LibRandom/include/
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

add_subdirectory (LibUtil)
add_subdirectory (Gnuplot)
add_subdirectory (LibBotHardware)
add_subdirectory (LibOdoCalib)
add_subdirectory (LibGps)
add_subdirectory (LibRobot)
add_subdirectory (LibMlsmap)
add_subdirectory (LibPath)
add_subdirectory (LibLaserProcessing)
add_subdirectory (LibRandom)
add_subdirectory (LibGeneric)
add_subdirectory (LibClustering)
add_subdirectory (LibRosUtil)
