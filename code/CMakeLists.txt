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

if( cisst_FOUND )

  # load cisst configuration
  include (${CISST_USE_FILE})

  # create/configure file for find_package (sawNDITracker)
  set(sawTrajectories_INCLUDE_DIR "${sawTrajectories_SOURCE_DIR}/include")
  set(sawTrajectories_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set(sawTrajectories_LIBRARIES sawTrajectories)

  include_directories (${sawTrajectories_INCLUDE_DIR})
  set (sawTrajectories_HEADER_DIR "${sawTrajectories_INCLUDE_DIR}/sawTrajectories")

  set(
    HEADER_FILES
    ${sawTrajectories_HEADER_DIR}/osaTrajectory.h
    ${sawTrajectories_HEADER_DIR}/mtsTrajectory.h )

  set(
    SOURCE_FILES
    osaTrajectory.cpp
    mtsTrajectory.cpp )

  add_library( sawTrajectories ${HEADER_FILES} ${SOURCE_FILES} )
  set_property (TARGET sawTrajectories PROPERTY FOLDER "sawTrajectories")
  cisst_target_link_libraries( sawTrajectories ${REQUIRED_CISST_LIBRARIES} )
  set_property (TARGET sawTrajectories PROPERTY FOLDER "sawTrajectories")

  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/sawTrajectoriesConfig.cmake.in"
    "${sawTrajectories_BINARY_DIR}/sawTrajectoriesConfig.cmake"
    @ONLY@ )

  # Install targets for headers and library
  install (DIRECTORY
           ${sawTrajectories_INCLUDE_DIR}/sawTrajectories
           DESTINATION include
           PATTERN .svn EXCLUDE)

  install( TARGETS sawTrajectories
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)

  install( FILES "${sawTrajectories_BINARY_DIR}/sawTrajectoriesConfig.cmake"
    DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

endif( cisst_FOUND )
