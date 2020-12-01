
IF("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   MESSAGE(FATAL_ERROR "CMake >= 2.6.0 required")
ENDIF()
CMAKE_POLICY(PUSH)
CMAKE_POLICY(VERSION 2.6)

if (TARGET pantiltplugin)
    GET_TARGET_PROPERTY(pantiltplugin_location pantiltplugin LOCATION)
endif()
if (NOT @CMAKE_PROJECT_NAME@_SOURCE_DIR AND NOT pantiltplugin_location AND NOT TARGET pantiltplugin)
    # Commands may need to know the format version.
    SET(CMAKE_IMPORT_FILE_VERSION 1)

    # Compute the installation prefix relative to this file.
    GET_FILENAME_COMPONENT(_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
    GET_FILENAME_COMPONENT(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
    GET_FILENAME_COMPONENT(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)

    # Create imported target pantiltplugin
    ADD_LIBRARY(pantiltplugin SHARED IMPORTED)
    if (WIN32)
        find_file(IMPORTLIB pantiltplugin.lib PATHS "${_IMPORT_PREFIX}/lib/Plugins"
                  PATH_SUFFIXES  Release RelWithDebInfo MinSizeRel Debug NO_DEFAULT_PATHS)
        SET_TARGET_PROPERTIES(pantiltplugin PROPERTIES
                              IMPORTED_LOCATION "${_IMPORT_PREFIX}/lib/Plugins/pantiltplugin.dll"
                              IMPORTED_IMPLIB   "${IMPORTLIB}")
        unset(IMPORTLIB CACHE)
    elseif(APPLE)
        SET_TARGET_PROPERTIES(pantiltplugin PROPERTIES
                              IMPORTED_LOCATION "${_IMPORT_PREFIX}/lib/Plugins/libpantiltplugin.@PANTILTPLUGIN_SOVERSION@.dylib"
                              IMPORTED_SONAME "libpantiltplugin.@PANTILTPLUGIN_SOVERSION@.dylib")
    else()
        SET_TARGET_PROPERTIES(pantiltplugin PROPERTIES
                              IMPORTED_LOCATION "${_IMPORT_PREFIX}/lib/Plugins/libpantiltplugin.so.@PANTILTPLUGIN_SOVERSION@"
                              IMPORTED_SONAME "libpantiltplugin.so.@PANTILTPLUGIN_SOVERSION@")
    endif()
    SET_TARGET_PROPERTIES(pantiltplugin PROPERTIES IMPORTED_LINK_INTERFACE_LIBRARIES "workspace")

    # Commands beyond this point should not need to know the version.
    SET(CMAKE_IMPORT_FILE_VERSION)
endif()
UNSET(pantiltplugin_location)

CMAKE_POLICY(POP)
