
##MESSAGE( STATUS "Installing library to " "${CMAKE_INSTALL_PREFIX}/lib")
##install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION "lib/")

##MESSAGE( STATUS "Installing include folders:" ${ApproxMVBB_INCLUDE_DIR} " to ${CMAKE_INSTALL_PREFIX}/include")
##install(DIRECTORY  ${ApproxMVBB_INCLUDE_DIR}  DESTINATION "include/" 
        ##FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.cmake" EXCLUDE 
        ##PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)


##MESSAGE( STATUS "Installing external include folders:" ${ApproxMVBB_EXTERNAL_INCLUDE_DIR} " to ${CMAKE_INSTALL_PREFIX}/include/ApproxMVBB")
##install(DIRECTORY  ${ApproxMVBB_EXTERNAL_INCLUDE_DIR}
        ##DESTINATION "include/ApproxMVBB"  
        ##FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.cmake" EXCLUDE 
        ##PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
        
        

#MESSAGE( STATUS "Installing config file:" ${ApproxMVBB_CONFIG_FILE} " to ${CMAKE_INSTALL_PREFIX}/include/ApproxMVBB/Config")
SET(ApproxMVBB_BUILD_LIBRARY_0or1 0) # Turn off library build flag, we are installing -> export symbols stuff
#configure_file (
  #"${${ApproxMVBBProjectName}_SOURCE_DIR}/include/ApproxMVBB/Config/Config.hpp.in.cmake"
  #"${CMAKE_INSTALL_PREFIX}/include/ApproxMVBB/Config"
#)