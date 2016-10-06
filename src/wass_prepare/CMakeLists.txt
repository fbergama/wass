cmake_minimum_required(VERSION 3.1)
project( wass_prepare )

add_definitions(-DBOOST_LOG_DYN_LINK)
include_directories( "${CMAKE_CURRENT_BINARY_DIR}/../include/"
                     "../include/"
                      ${Boost_INCLUDE_DIRS}
                      ${OpenCV_INCLUDE_DIRS}
                    )



if(MSVC)
link_directories( ${Boost_LIBRARY_DIR_RELEASE} )
endif()

add_executable( ${PROJECT_NAME} wass_prepare.cpp )

if(MSVC)
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS}  )
else()
target_link_libraries( ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} ${Boost_LIBRARIES}   )
endif()

INSTALL( TARGETS ${PROJECT_NAME} DESTINATION bin PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )


