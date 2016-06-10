cmake_minimum_required(VERSION 3.1)
project( OpenSURF )

include_directories(
                    ${Boost_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    )

add_library( ${PROJECT_NAME} fasthessian.cpp
                             fasthessian.h
                             integral.cpp
                             integral.h
                             ipoint.cpp
                             ipoint.h
                             kmeans.h
                             responselayer.h
                             surf.cpp
                             surf.h
                             surflib.h
                             utils.cpp
                             utils.h
                             )

INSTALL( TARGETS ${PROJECT_NAME} DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )

SET_TARGET_PROPERTIES( ${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION} SOVERSION ${${PROJECT_NAME}_SOVERSION} CLEAN_DIRECT_OUTPUT 1 OUTPUT_NAME ${PROJECT_NAME} )
INSTALL( TARGETS ${PROJECT_NAME} DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
INSTALL( FILES "surflib.h"
         DESTINATION "OpenSURF/include" PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ )


