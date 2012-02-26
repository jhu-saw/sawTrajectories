
set( 
  REQUIRED_CISST_LIBRARIES 
  cisstCommon
  cisstVector
  cisstOSAbstraction
  cisstMultiTask
  cisstParameterTypes
  cisstRobot 
  cisstNumerical )

find_package( cisst REQUIRED ${REQUIRED_CISST_LIBRARIES} )

if( cisst_FOUND )

  # load cisst configuration
  include (${CISST_USE_FILE})
  
  set( 
    HEADER_FILES 
    ../osaTrajectory.h

    ../mtsTrajectory.h )

  
  set( 
    SOURCE_FILES 
    osaTrajectory.cpp

    mtsTrajectory.cpp )

  
  add_library( sawTrajectories ${HEADER_FILES} ${SOURCE_FILES} )
  set_property (TARGET sawTrajectories PROPERTY FOLDER "sawTrajectories")
  cisst_target_link_libraries( sawTrajectories ${REQUIRED_CISST_LIBRARIES} )

  # create/configure file for find_package (sawNDITracker)
  set(sawTrajectories_INCLUDE_DIR "${sawComponents_SOURCE_DIR}" "${sawComponents_BINARY_DIR}")
  set(sawTrajectories_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set(sawTrajectories_LIBRARIES sawTrajectories)

  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/sawTrajectoriesConfig.cmake.in"
    "${sawComponents_BINARY_DIR}/sawTrajectoriesConfig.cmake"
    @ONLY@ )

  # Install targets for headers and library
  install( FILES ${HEADER_FILES}
    DESTINATION include/sawTrajectories)

  install( TARGETS sawTrajectories
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

  install( FILES "${sawComponents_BINARY_DIR}/sawTrajectoriesConfig.cmake"
    DESTINATION ${CISST_CMAKE_MODULES_INSTALL_SUFFIX}/components)

endif( cisst_FOUND )
