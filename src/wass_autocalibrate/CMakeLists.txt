cmake_minimum_required(VERSION 3.1)
project( wass_autocalibrate )

SET( SBA_ROOT "${PROJECT_SOURCE_DIR}/../../ext/sba-1.6")
MESSAGE( STATUS "${SBA_ROOT}" )

# Search for SBA library
find_path(SBA_INCLUDE_DIR sba.h HINTS ${SBA_ROOT} )
if( NOT SBA_INCLUDE_DIR )

    MESSAGE( STATUS "SBA include dir not found, downloading...")
    SET( SBA_PACKAGE "${SBA_ROOT}/../sba-1.6.tgz" )
    file(DOWNLOAD "http://www.dais.unive.it/wass/files/sba-1.6.tar" ${SBA_PACKAGE}  )
    execute_process(COMMAND ${CMAKE_COMMAND} -E tar xf ${SBA_PACKAGE} WORKING_DIRECTORY "${SBA_ROOT}/../")
    MESSAGE( STATUS "Compiling SBA...")
    execute_process(COMMAND "make" WORKING_DIRECTORY "${SBA_ROOT}" OUTPUT_QUIET ERROR_QUIET )

    # try to find it again
    find_path(SBA_INCLUDE_DIR sba.h HINTS ${SBA_ROOT} )
    if( NOT SBA_INCLUDE_DIR )
        MESSAGE( FATAL_ERROR "SBA include dir not found. I tried to download it for you but something went wrong. Please set SBA_ROOT path variable manually")
    else()
        MESSAGE( STATUS "SBA_INCLUDE_DIR: ${SBA_INCLUDE_DIR}")
    endif()
endif()


find_library( SBA_LIBRARY LIBRARY NAMES sba HINTS ${SBA_INCLUDE_DIR} )
if( NOT SBA_LIBRARY )
    MESSAGE( FATAL_ERROR "SBA library not found. I've tried to compile it for you but sumething went wrong. Please manually compile SBA library and place it into ${SBA_INCLUDE_DIR}")
else()

    MESSAGE( STATUS "SBA_LIBRARY: ${SBA_LIBRARY}")

if( MSVC )
	# Use the CMAKE version of clapack. See: http://www.netlib.org/clapack/
	find_package( clapack )
else()
    find_package( LAPACK )
    if( NOT LAPACK_FOUND )
        MESSAGE( FATAL_ERROR "Lapack (required by sba) not found.")
    else()
        MESSAGE( STATUS "Lapack found: ${LAPACK_LIBRARIES}")
    endif()
endif()

endif()


if( DISABLE_BOOST_LOG )
else()
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DWASS_USE_BOOST_LOG)
endif()

include_directories( "${CMAKE_CURRENT_BINARY_DIR}/../include/"
                     "../include/"
                     "../wass_lib"
                     "../OpenSURF"
                      ${INCFG_DIR}
                      ${Boost_INCLUDE_DIRS}
                      ${OpenCV_INCLUDE_DIRS}
                      ${SBA_INCLUDE_DIR}
                    )



add_executable( ${PROJECT_NAME} wass_autocalibrate.cpp
                                sba_driver.cpp
                                sba_driver.h
                                ${INCFG_LIBRARY}
              )


if(MSVC)
    target_link_libraries( ${PROJECT_NAME} wass_lib ${OpenCV_LIBS} Boost::filesystem ${SBA_LIBRARY} ${LAPACK_LIBRARIES} f2c blas lapack  )
else()
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    target_link_libraries( ${PROJECT_NAME} Threads::Threads wass_lib ${OpenCV_LIBS} ${Boost_LIBRARIES} ${SBA_LIBRARY} ${LAPACK_LIBRARIES} )
endif()

INSTALL( TARGETS ${PROJECT_NAME} DESTINATION bin PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )


