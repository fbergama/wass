CMAKE_MINIMUM_REQUIRED( VERSION 3.1 )
PROJECT( WASS )

SET( WASS_VERSION_STR "1.21" )


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
set(CMAKE_CXX_STANDARD 11)


# Setup default installation dir
#
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
get_filename_component( INSTALL_DIR_ABS "${PROJECT_SOURCE_DIR}/../dist" ABSOLUTE)
SET( CMAKE_INSTALL_PREFIX ${INSTALL_DIR_ABS} CACHE PATH "Install prefix" FORCE )
ENDIF()

# Set default CMake build type
if( NOT CMAKE_BUILD_TYPE )
    set( CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE )
endif()


####################################
# RPATH HANDLING
####################################
set(CMAKE_MACOSX_RPATH 1)

# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

####################################

set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )


# Search for required dependencies
#
#
FIND_PACKAGE( OpenCV REQUIRED core imgproc calib3d highgui)
IF( OpenCV_FOUND )
    MESSAGE( STATUS "OpenCV Version ${OpenCV_VERSION} found.")
ENDIF()

IF( NOT Boost_INCLUDE_DIRS )
    set(Boost_USE_MULTITHREADED ON)
	IF( WIN32 )
		set(Boost_USE_STATIC_LIBS ON)
		#set(Boost_USE_STATIC_RUNTIME ON)
		set(Boost_COMPILER "-vc142")
		set(Boost_USE_STATIC_LIBS ON)
	ELSE()
		set(Boost_USE_STATIC_LIBS OFF)
		set(Boost_USE_STATIC_RUNTIME OFF)
	ENDIF()

	set(Boost_DEBUG ON)
	#set(Boost_DETAILED_FAILURE_MSG ON)
        #find_package(Boost 1.65.0 REQUIRED COMPONENTS system log_setup log date_time thread program_options filesystem )
    find_package(Boost 1.65.0 REQUIRED COMPONENTS system date_time thread program_options filesystem )
    IF( NOT Boost_FOUND )
        MESSAGE( FATAL_ERROR " Boost was not found. Please set BOOST_ROOT path variable")
    ENDIF()
ENDIF()

# Configure file
execute_process( COMMAND "git"  "describe" "--all" "--long" WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" RESULT_VARIABLE COMMAND_RETURN_CODE OUTPUT_VARIABLE WASS_GIT_VERSION_STR ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE )
#MESSAGE( STATUS "Command result: ${COMMAND_RETURN_CODE}")
IF( ${COMMAND_RETURN_CODE} EQUAL 0 )
    SET( WASS_VERSION_STR "${WASS_VERSION_STR}_${WASS_GIT_VERSION_STR}" )
ELSE()
    MESSAGE( STATUS "No git repository detected, keeping the default version string")
ENDIF()

MESSAGE( STATUS "WASS generated version string: ${WASS_VERSION_STR}")

SET( WASS_BUILD_ENV_STR "${CMAKE_SYSTEM} - ${CMAKE_CXX_COMPILER_ID}")
SET( WASS_BUILD_TYPE_STR "${CMAKE_BUILD_TYPE}")
SET( WASS_OCV_VERSION_STR "${OpenCV_VERSION}")

configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/include/wassglobal.hpp.in" "${CMAKE_CURRENT_BINARY_DIR}/include/wassglobal.hpp" )



##

find_path( INCFG_DIR NAMES incfg.hpp PATHS "${PROJECT_SOURCE_DIR}/../ext/incfg/" NO_DEFAULT_PATH)

IF( NOT INCFG_DIR )
    MESSAGE( FATAL_ERROR " Incfg library not found, please run git submodule init/update" )
ELSE()
    SET( INCFG_LIBRARY "${INCFG_DIR}/incfg.cpp" CACHE FILEPATH "incfg library source" FORCE )
    MESSAGE( STATUS "Incfg library found!")
ENDIF()


# Add subprojects
#
OPTION( BUILD_WASS_PREPARE "Build wass_prepare executable" ON )
OPTION( BUILD_WASS_MATCH   "Build wass_match executable" ON )
OPTION( BUILD_WASS_AUTOCALIBRATE  "Build wass_autocalibrate executable" ON )
OPTION( BUILD_WASS_STEREO "Build wass_stereo executable" ON )
OPTION( DISABLE_BOOST_LOG "Disable Boost Log subsystem" ON )


add_subdirectory( wass_lib )

IF( BUILD_WASS_PREPARE )
    ADD_SUBDIRECTORY( wass_prepare )
ENDIF()

IF( BUILD_WASS_MATCH )
    ADD_SUBDIRECTORY( wass_match )
ENDIF()

IF( BUILD_WASS_AUTOCALIBRATE )
    ADD_SUBDIRECTORY( wass_autocalibrate )
ENDIF()

IF( BUILD_WASS_STEREO )
    ADD_SUBDIRECTORY( wass_stereo )
ENDIF()


####################################

IF( WIN32 )
ADD_SUBDIRECTORY( package_win32 )
ENDIF()


####################################
# Build overview

MESSAGE( STATUS "------------------------------------------------- ")
MESSAGE( STATUS " Build overview: ")
MESSAGE( STATUS "------------------------------------------------- ")
MESSAGE( STATUS "BUILD_WASS_PREPARE:         ${BUILD_WASS_PREPARE} ")
MESSAGE( STATUS "BUILD_WASS_MATCH:           ${BUILD_WASS_MATCH} ")
MESSAGE( STATUS "BUILD_WASS_AUTOCALIBRATE:   ${BUILD_WASS_AUTOCALIBRATE} ")
MESSAGE( STATUS "BUILD_WASS_STEREO:          ${BUILD_WASS_STEREO} ")
MESSAGE( STATUS "CMAKE_BUILD_TYPE:           ${CMAKE_BUILD_TYPE} ")
MESSAGE( STATUS "CMAKE_INSTALL_PREFIX:       ${CMAKE_INSTALL_PREFIX} ")
MESSAGE( STATUS "OpenCV_VERSION:             ${OpenCV_VERSION} ")
MESSAGE( STATUS "Boost_VERSION:              ${Boost_VERSION} ")
MESSAGE( STATUS "DISABLE_BOOST_LOG:          ${DISABLE_BOOST_LOG} ")
MESSAGE( STATUS "------------------------------------------------- ")
MESSAGE( STATUS "${OpenCV_LIBS} ")
MESSAGE( STATUS "${OpenCV_INCLUDE_DIRS} ")
MESSAGE( STATUS "${Boost_LIBRARIES} ")
