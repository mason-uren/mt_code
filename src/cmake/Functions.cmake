# Functions.cmake
#
# A collection of ultility functions to be used by root and chilren applications.
#
# This will define the following variables
#
#    load_parallel_module
#
# Author: Mason U'Ren

cmake_minimum_required(VERSION 3.5)

function(load_parallel_module MODULE_TARGET)
	if (NOT TARGET ${MODULE_TARGET}) # Check if module has already been built
		#get_filename_component(${MODULE_TARGET}_BIN ${PROJECT_SOURCE_DIR}/../${MODULE_TARGET}/build ABSOLUTE)
		add_subdirectory(../${MODULE_TARGET} ${MODULE_TARGET}_BIN)
	else()
		# TODO - how to link module if already found/built

		message(STATUS ======================)
		message(STATUS "${MODULE_TARGET} - Library previously sourced.")
		message(STATUS ======================)
	endif()
endfunction(load_parallel_module)

function(exclude_target TARGET_NAME)
	set_target_properties(
		${TARGET_NAME} PROPERTIES 
			EXCLUDE_FROM_DEFAULT_BUILD TRUE # Needed for MSVS (double setting is not an issue)
			EXCLUDE_FROM_ALL		   TRUE
	)
endfunction()