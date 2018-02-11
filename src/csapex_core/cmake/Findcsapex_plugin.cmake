macro(find_csapex_plugin name library  headerfile)
 	MESSAGE(STATUS "Searching for CS::APEX plugin " ${name})
	
        if(csapex_plugin_${name}_INCLUDE_DIRS)
		# Built in this project
		set(${name}_FOUND True)
			
                set(${name}_INCLUDE_DIRS ${csapex_plugin_${name}_INCLUDE_DIRS})
		set(${name}_LIBRARIES ${csapex_plugin_${name}_LIBRARIES})
	else()
                FIND_PATH(${name}_INCLUDE_DIRS NAMES ${name}/${headerfile} PATH_SUFFIXES ${SUFFIX_FOR_INCLUDE_PATH} PATHS
		# Look in other places.
		  ${CSAPEX_DIR_SEARCH}

		# Help the user find it if we cannot.
		  DOC "The ${CSAPEX_INCLUDE_PATH_DESCRIPTION}"
		)
		
		
		FIND_LIBRARY(${name}_LIBRARIES NAMES ${library} PATHS
                  ${csapex_INCLUDE_DIRS}/../lib
                  ${csapex_INCLUDE_DIRS}/../bin
		  ${CSAPEX_DIR_SEARCH}
		)
		
                if(${name}_INCLUDE_DIRS AND ${name}_LIBRARIES)
			set(${name}_FOUND True)
			set(${name}_LIBRARIES)
		else()
			set(${name}_FOUND False)
		
		  #IF(${name}_FIND_QUIETLY)
		#	MESSAGE(STATUS "${name} was not found.")
		#  ELSE(${name}_FIND_QUIETLY)
	#		IF(${name}_FIND_REQUIRED)
	#		  MESSAGE(FATAL_ERROR "${name} was not found. ")
	#		ENDIF(${name}_FIND_REQUIRED)
	#	  ENDIF(${name}_FIND_QUIETLY)

			include(FindPackageHandleStandardArgs)
                        find_package_handle_standard_args(${name} DEFAULT_MSG ${name}_LIBRARIES ${name}_INCLUDE_DIRS)
		endif()
	endif()
	
endmacro()
