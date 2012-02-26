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
  include( ${CISST_USE_FILE} )

  cisst_find_component(sawTrajectories REQUIRED )
  cisst_find_component(sawKeyboard REQUIRED )

  if( sawTrajectories_FOUND AND sawKeyboard_FOUND)

    link_directories(${sawKeyboard_LIBRARY_DIR})

    add_executable(osaIKExample         osaIKExample.cpp )
    set_property (TARGET osaIKExample PROPERTY FOLDER "sawTrajectories")
    add_executable(mtsTrajectoryExample mtsTrajectoryExample.cpp )
    set_property (TARGET mtsTrajectoryExample PROPERTY FOLDER "sawTrajectories")

    target_link_libraries( osaIKExample          sawTrajectories )
    target_link_libraries( mtsTrajectoryExample  sawTrajectories sawKeyboard )

    cisst_target_link_libraries(osaIKExample         ${REQUIRED_CISST_LIBRARIES} )
    cisst_target_link_libraries(mtsTrajectoryExample ${REQUIRED_CISST_LIBRARIES} )

  endif( sawTrajectories_FOUND AND sawKeyboard_FOUND )

endif( cisst_FOUND  )




