# FindLBFGS.cmake
#
# Find the header-only, template file(s) resposible for our minimization algorithm
#
# This will define the following variables
#
#    LBFGS_FOUND
#    LBFGS_DIR
#    LBFGS_INCLUDE_DIRS
#
# Author: Mason U'Ren

######################################
# Set package specific variables
######################################
set(PACKAGE_TARGET_NAME "LBFGSpp")
set(TARGET_PATH ${CMAKE_PREFIX_PATH}/${PACKAGE_TARGET_NAME})

# Version Library
set(LIB_MAJOR_VERSION "1")
set(LIB_MINOR_VERSION "0")
set(LIB_PATCH_VERSION "1")
set(LIB_VERSION_STRING "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_PATCH_VERSION}")

# Header Files
set(INCLUDE_HDRS
        LBFGS.h
		LBFGSB.h
		LBFGSpp/BFGSMat.h
		LBFGSpp/BKLDLT.h
		LBFGSpp/Cauchy.h
		LBFGSpp/LineSearchBacktracking.h
		LBFGSpp/LineSearchBracketing.h
		LBFGSpp/LineSearchMoreThuente.h
		LBFGSpp/LineSearchNocedalWright.h
		LBFGSpp/Param.h
		LBFGSpp/SubspaceMin.h
)

######################################
# Package creation
######################################
# Set package version...
set(${PACKAGE_TARGET_NAME}_VERSION ${LIB_VERSION_STRING})

# Set root _DIR...
set(${PACKAGE_TARGET_NAME}_DIR ${TARGET_PATH})

#message("TARGET_PATH=${TARGET_PATH}")
# Find includes...
foreach(header ${INCLUDE_HDRS})
	find_path(
		${header}_INC
		NAMES
			${header}
		PATHS
			${TARGET_PATH}
		PATH_SUFFIXES
		    include
	)

	# Verify header was found
	if (NOT ${header}_INC)
		message("${header} not found!")
	else()
		# Add element
		if (NOT ${header}_INC IN_LIST ${PACKAGE_TARGET_NAME}_INCLUDE_DIRS)
			list(APPEND ${PACKAGE_TARGET_NAME}_INCLUDE_DIRS ${${header}_INC})
		endif()
	endif()
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
        ${PACKAGE_TARGET_NAME}
        REQUIRED_VARS
            ${PACAKGE_TARGET_NAME}_DIR
            ${PACKAGE_TARGET_NAME}_INCLUDE_DIRS
        VERSION_VAR
            ${PACKAGE_TARGET_NAME}_VERSION
)

mark_as_advanced(
		${PACKAGE_TARGET_NAME}_FOUND
		${PACAKGE_TARGET_NAME}_DIR
		${PACKAGE_TARGET_NAME}_INCLUDE_DIRS
		${PACKAGE_TARGET_NAME}_VERSION
)