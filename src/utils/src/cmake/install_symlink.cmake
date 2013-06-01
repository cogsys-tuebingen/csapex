if(NOT RELATIVE_DIRECTORY)
    set(RELATIVE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

string(REGEX REPLACE ".*/src/([^/]+)" "\\1" lib ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB_RECURSE headers RELATIVE ${RELATIVE_DIRECTORY} *.h *.hpp)
foreach(h ${headers})
    get_filename_component(dir ${h} PATH)

    execute_process(COMMAND mkdir -p ${PROJECT_SOURCE_DIR}/include/utils/${lib}/${dir}
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

    execute_process(COMMAND ln -sf ${RELATIVE_DIRECTORY}/${h} -t ${PROJECT_SOURCE_DIR}/include/utils/${lib}/${dir}
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()

set(RELATIVE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
