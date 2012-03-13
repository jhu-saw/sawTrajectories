#
# $Id: CMakeLists.txt 3485 2012-02-26 06:08:41Z jbohren1 $
#
# (C) Copyright 2011-2012 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

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

if( cisst_FOUND_AS_REQUIRED )

  # load cisst configuration
  include( ${CISST_USE_FILE} )

  cisst_find_saw_component (sawTrajectories REQUIRED)
  cisst_find_saw_component (sawKeyboard REQUIRED)

  if( sawTrajectories_FOUND AND sawKeyboard_FOUND)

    include_directories (${sawKeyboard_INCLUDE_DIR} ${sawTrajectories_INCLUDE_DIR})
    link_directories (${sawKeyboard_LIBRARY_DIR} ${sawTrajectories_LIBRARY_DIR})

    set (sawTrajectories_EXAMPLES
         osaIKExample
         mtsTrajectoryExample)

    foreach (_example ${sawTrajectories_EXAMPLES})
      add_executable (${_example} ${_example}.cpp)
      target_link_libraries (${_example} ${sawTrajectories_LIBRARIES})
      cisst_target_link_libraries (${_example} ${REQUIRED_CISST_LIBRARIES})
      set_property (TARGET ${_example} PROPERTY FOLDER "sawTrajectories")
    endforeach ()

    # examples that also need sawKeyboard
    target_link_libraries (mtsTrajectoryExample ${sawKeyboard_LIBRARIES})

  endif( sawTrajectories_FOUND AND sawKeyboard_FOUND )

endif( cisst_FOUND_AS_REQUIRED )
