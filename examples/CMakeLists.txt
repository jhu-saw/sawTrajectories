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
find_package(sawTrajectories REQUIRED HINTS "${cisst_saw_BINARY_DIR}/saw/components")

if( cisst_FOUND AND sawTrajectories_FOUND )

  # load cisst configuration
  include( ${CISST_USE_FILE} )

  add_executable(osaIKExample         osaIKExample.cpp )
  set_property (TARGET osaIKExample PROPERTY FOLDER "sawTrajectories")
  add_executable(mtsTrajectoryExample mtsTrajectoryExample.cpp )
  set_property (TARGET mtsTrajectoryExample PROPERTY FOLDER "sawTrajectories")

  target_link_libraries( osaIKExample          sawTrajectories )
  target_link_libraries( mtsTrajectoryExample  sawTrajectories sawKeyboard )

  cisst_target_link_libraries(osaIKExample         ${REQUIRED_CISST_LIBRARIES} )
  cisst_target_link_libraries(mtsTrajectoryExample ${REQUIRED_CISST_LIBRARIES} )

endif( cisst_FOUND AND sawTrajectories_FOUND )




