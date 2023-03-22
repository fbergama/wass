CMAKE_MINIMUM_REQUIRED( VERSION 3.1 )

SET(PACK_NAME "wass_1.8_win32_x64")
get_filename_component( PACK_DIR "${CMAKE_INSTALL_PREFIX}/../${PACK_NAME}" ABSOLUTE )
MESSAGE( STATUS "Creating ${PACK_DIR} ")
file( MAKE_DIRECTORY ${PACK_DIR} )

MESSAGE( STATUS "Copying wass executables ")
execute_process( COMMAND ${CMAKE_COMMAND} -E remove -f "${CMAKE_INSTALL_PREFIX}/${PACK_NAME}.zip" )
execute_process( COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_INSTALL_PREFIX} "${PACK_DIR}/dist" )

MESSAGE( STATUS "Copying WASSjs")
execute_process( COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_INSTALL_PREFIX}/../WASSjs" "${PACK_DIR}/WASSjs" )

MESSAGE( STATUS "Copying test")
execute_process( COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_INSTALL_PREFIX}/../test" "${PACK_DIR}/test" )

MESSAGE( STATUS "Copying matlab")
execute_process( COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_INSTALL_PREFIX}/../matlab" "${PACK_DIR}/matlab" )

MESSAGE( STATUS "Copying additional files")
execute_process( COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_INSTALL_PREFIX}/../README.md" "${PACK_DIR}/README.md" )
execute_process( COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_INSTALL_PREFIX}/../COPYING" "${PACK_DIR}/COPYING" )

MESSAGE( STATUS "Removing node modules")
execute_process( COMMAND ${CMAKE_COMMAND} -E remove_directory "${PACK_DIR}/WASSjs/node_modules" )

MESSAGE( STATUS "Creating ZIP archive")
execute_process( COMMAND ${CMAKE_COMMAND} -E tar "cf" "${CMAKE_INSTALL_PREFIX}/${PACK_NAME}.zip" --format=zip ${PACK_DIR} WORKING_DIRECTORY "${PACK_DIR}/../" )

MESSAGE( STATUS "Removing temp folders")
execute_process( COMMAND ${CMAKE_COMMAND} -E remove_directory "${PACK_DIR}" )

MESSAGE( STATUS "All done!")
