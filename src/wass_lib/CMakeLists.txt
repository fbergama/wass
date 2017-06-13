cmake_minimum_required(VERSION 3.1)
project( wass_lib )

if( DISABLE_BOOST_LOG )
else()
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DWASS_USE_BOOST_LOG)
endif()

include_directories( "${CMAKE_CURRENT_BINARY_DIR}/../include/"
                     "../include/"
                      ${Boost_INCLUDE_DIRS}
                      ${OpenCV_INCLUDE_DIRS}
                      )

add_library( ${PROJECT_NAME} hires_timer.cpp
                             hires_timer.h
                             triangulate.hpp
                             epipolar.cpp
                             epipolar.h
                             )

INSTALL( TARGETS ${PROJECT_NAME} DESTINATION bin PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )

SET_TARGET_PROPERTIES( ${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION} SOVERSION ${${PROJECT_NAME}_SOVERSION} CLEAN_DIRECT_OUTPUT 1 OUTPUT_NAME ${PROJECT_NAME} )
INSTALL( TARGETS ${PROJECT_NAME} DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
INSTALL( FILES "triangulate.hpp"
               "hires_timer.h"
                DESTINATION "include" PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ )


