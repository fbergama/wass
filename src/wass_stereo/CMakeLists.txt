cmake_minimum_required(VERSION 3.1)
project( wass_stereo )

include(CheckIncludeFileCXX)
check_include_file_cxx( opencv2/optflow.hpp HAVE_OPENCV_OPTFLOW )
if(HAVE_OPENCV_OPTFLOW)
    #    add_definitions(-DWASS_ENABLE_OPTFLOW)
endif(HAVE_OPENCV_OPTFLOW)

if( DISABLE_BOOST_LOG )
else()
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DWASS_USE_BOOST_LOG)
endif()

include_directories( "${CMAKE_CURRENT_BINARY_DIR}/../include/"
                     "../include/"
                     "../wass_lib"
                      ${INCFG_DIR}
                      ${Boost_INCLUDE_DIRS}
                      ${OpenCV_INCLUDE_DIRS}
                      )


add_executable( ${PROJECT_NAME} wass_stereo.cpp
                                PovMesh.cpp
                                render.hpp
                                nanoflann.hpp
                                stereorectify.cpp
                                stereorectify.h
                                ${INCFG_LIBRARY}
                                )

if(MSVC)
    target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} Boost::filesystem wass_lib )
else()
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    target_link_libraries( ${PROJECT_NAME} Threads::Threads ${OpenCV_LIBS} ${Boost_LIBRARIES} wass_lib )
endif()

INSTALL( TARGETS ${PROJECT_NAME} DESTINATION bin PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )


