cmake_minimum_required(VERSION 3.1)
project( wass_autocalibrate )


# Search for SBA library
find_path(SBA_INCLUDE_DIR sba.h HINTS ${SBA_ROOT} )
if( NOT SBA_INCLUDE_DIR )
MESSAGE( FATAL_ERROR "SBA include dir not found. Please set SBA_ROOT path variable")
else()
MESSAGE( STATUS "SBA_INCLUDE_DIR: ${SBA_INCLUDE_DIR}")
find_library( SBA_LIBRARY LIBRARY NAMES sba HINTS ${SBA_INCLUDE_DIR} )
if( NOT SBA_LIBRARY )
MESSAGE( FATAL_ERROR "SBA library not found.")
else()
MESSAGE( STATUS "SBA_LIBRARY: ${SBA_LIBRARY}")

find_package( LAPACK )
if( NOT LAPACK_FOUND )
    MESSAGE( FATAL_ERROR "Lapack (required by sba) not found.")
else()
    MESSAGE( STATUS "Lapack found: ${LAPACK_LIBRARIES}")
endif()


endif()
endif()


add_definitions(-DBOOST_LOG_DYN_LINK)
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


target_link_libraries( ${PROJECT_NAME} wass_lib OpenSURF ${OpenCV_LIBS} ${Boost_LIBRARIES} ${SBA_LIBRARY} ${LAPACK_LIBRARIES} )
INSTALL( TARGETS ${PROJECT_NAME} DESTINATION bin PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )


