cmake_minimum_required(VERSION 3.8.2)

project(csapex_qt CXX)

include (GenerateExportHeader)

find_package(catkin REQUIRED COMPONENTS csapex_core csapex_remote)

find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL REQUIRED)

find_package(OpenGL REQUIRED)

# make sure the devel include exists, there we will generate the export headers
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

catkin_package(
   INCLUDE_DIRS
      include
      external/qxt5/include
      ${CATKIN_DEVEL_PREFIX}/include
      ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
   LIBRARIES
      ${PROJECT_NAME}
      Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL
   CATKIN_DEPENDS
      csapex_core
      csapex_remote
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_add_resources(QT_RESOURCES res/csapex_qt_resources.qrc)
qt5_wrap_ui(QT_UI
    ui/box.ui
    ui/csapex_window.ui
    ui/designer.ui)


add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/external)


include_directories(
    include
    external/qxt5/include
    ${OPENGL_INCLUDE_DIR}
    ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)


#
# BUILD THE LIBRARIES
#



file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/csapex/view/*.h*)

if(NOT WIN32)
    # for visual studio, we want the headers in the library
    # however, with ubuntu that doesn't compile due to auto moc'ing
    #add_custom_target(${PROJECT_NAME}_headers SOURCES ${${PROJECT_NAME}_HEADERS})
    #set(${PROJECT_NAME}_HEADERS)
endif()

# Library for the app itself
add_library(${PROJECT_NAME} SHARED

    src/view/csapex_view_core.cpp
    src/view/csapex_view_core_impl.cpp
    src/view/csapex_view_core_proxy.cpp
    src/view/csapex_window.cpp
    src/view/gui_exception_handler.cpp

    src/view/model/thread_group_table_model.cpp
    src/view/model/thread_group_profiling_model.cpp

    src/view/designer/designer.cpp
    src/view/designer/designer_options.cpp
    src/view/designer/designer_scene.cpp
    src/view/designer/designer_styleable.cpp
    src/view/designer/graph_view.cpp
    src/view/designer/graph_view_context_menu.cpp
    src/view/designer/designerio.cpp
    src/view/designer/drag_io.cpp
    src/view/designer/drag_io_handler.cpp
    src/view/designer/fulcrum_handle.cpp
    src/view/designer/fulcrum_widget.cpp
    src/view/designer/tutorial_tree_model.cpp

    src/view/node/node_adapter.cpp
    src/view/node/default_node_adapter.cpp
    src/view/node/resizable_node_adapter.cpp
    src/view/node/node_adapter_builder.cpp
    src/view/node/node_adapter_factory.cpp
    src/view/node/node_filter_proxy_model.cpp
    src/view/node/parameter_context_menu.cpp
    src/view/node/box.cpp
    src/view/node/sticky_note_box.cpp
    src/view/node/node_statistics.cpp

    src/view/param/param_adapter.cpp
    src/view/param/range_param_adapter.cpp
    src/view/param/value_param_adapter.cpp

    src/view/param/param_adapter_builder.cpp
    src/view/param/param_adapter_factory.cpp
    src/view/param/trigger_param_adapter.cpp
    src/view/param/range_param_adapter.cpp
    src/view/param/interval_param_adapter.cpp
    src/view/param/set_param_adapter.cpp
    src/view/param/bitset_param_adapter.cpp
    src/view/param/value_param_adapter.cpp
    src/view/param/path_param_adapter.cpp
    src/view/param/color_param_adapter.cpp

    src/view/param/output_progress_param_adapter.cpp
    src/view/param/output_text_param_adapter.cpp

    src/view/utility/context_menu_handler.cpp
    src/view/utility/clipboard.cpp
    src/view/utility/cpu_affinity_delegate.cpp
    src/view/utility/html_delegate.cpp
    src/view/utility/node_list_generator.cpp
    src/view/utility/qobserver.cpp
    src/view/utility/qsignal_bridges.cpp
    src/view/utility/qsignal_relay.cpp
    src/view/utility/qt_helper.cpp
    src/view/utility/qwrapper.cpp
    src/view/utility/snippet_list_generator.cpp
    src/view/utility/thread_group_profiling_delegate.cpp
    src/view/utility/widget_picker.cpp

    src/view/widgets/tracing_legend.cpp
    src/view/widgets/tracing_timeline.cpp
    src/view/widgets/tracing_timeline_item.cpp
    src/view/widgets/completed_line_edit.cpp
    src/view/widgets/box_dialog.cpp
    src/view/widgets/search_dialog.cpp
    src/view/widgets/rewiring_dialog.cpp
    src/view/widgets/csapex_splashscreen.cpp
    src/view/widgets/doublespanslider.cpp
    src/view/widgets/minimap_widget.cpp
    src/view/widgets/port.cpp
    src/view/widgets/meta_port.cpp
    src/view/widgets/port_panel.cpp
    src/view/widgets/profiling_widget.cpp
    src/view/widgets/qdouble_slider.cpp
    src/view/widgets/qint_slider.cpp
    src/view/widgets/screenshot_dialog.cpp
    src/view/widgets/movable_graphics_proxy_widget.cpp
    src/view/widgets/thread_group_profiling_widget.cpp
    src/view/widgets/message_preview_widget.cpp
    src/view/widgets/notification_widget.cpp
    src/view/widgets/cpu_affinity_widget.cpp

    #${${PROJECT_NAME}_HEADERS}

    ${QT_RESOURCES}

    ${QT_UI}
)

generate_export_header( ${PROJECT_NAME}
             BASE_NAME ${PROJECT_NAME}
             EXPORT_MACRO_NAME CSAPEX_QT_EXPORT
             EXPORT_FILE_NAME ${PROJECT_NAME}/export.h
             STATIC_DEFINE ${PROJECT_NAME}_BUILT_AS_STATIC
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}

    qxt5

    Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${YAML-CPP_LIBRARY}
    ${TinyXML_LIBRARIES}
    ${class_loader_LIBRARIES}
    ${OPENGL_LIBRARIES}
)

set_target_properties(${PROJECT_NAME} PROPERTIES AUTOMOC TRUE)


#
# CFG
#
add_custom_target(${PROJECT_NAME}_cfg ALL DEPENDS cfg/style.css)
add_custom_command(TARGET ${PROJECT_NAME}_cfg POST_BUILD
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/setup.sh
        ${CATKIN_DEVEL_PREFIX}
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)


#
# INSTALL
#


set(${PROJECT_NAME}_EXPORT_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})
set(${PROJECT_NAME}_EXPORT_FILE ${${PROJECT_NAME}_EXPORT_DIR}/export.h)

file(COPY ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/export.h
    DESTINATION ${${PROJECT_NAME}_EXPORT_DIR})
install(FILES ${${PROJECT_NAME}_EXPORT_FILE}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY include/csapex/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/csapex)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
    