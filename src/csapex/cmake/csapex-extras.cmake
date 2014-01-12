find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtOpenGL)
include(${QT_USE_FILE})

add_definitions(-W -Wall -Wno-unused-parameter -fno-strict-aliasing -Wno-unused-function)
add_definitions(-g -rdynamic)

set(CMAKE_BUILD_TYPE Debug)

function(csapex_moc_all)
    set(p ${ARGN})
    list(REMOVE_AT p 0)
    set(regex)
    foreach(r ${p})
        list(APPEND regex "${r}")
    endforeach()

    file(GLOB_RECURSE T RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS ${regex})
    set(S)
    foreach(header ${T})
        file(STRINGS "${header}" lines REGEX "Q_OBJECT")
        if(lines)
            list(APPEND S "${header}")
        endif()
    endforeach()

    QT4_WRAP_CPP(S ${S})
    set(${ARGV0} ${S} PARENT_SCOPE)
endfunction(csapex_moc_all)
