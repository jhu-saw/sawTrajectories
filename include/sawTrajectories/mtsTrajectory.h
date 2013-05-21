/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: osaTrajectory.h 4202 2013-05-17 15:39:06Z adeguet1 $

  Author(s):  Simon Leonard
  Created on: 2012

  (C) Copyright 2012-2013 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsTrajectory_h
#define _mtsTrajectory_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsVector.h>
#include <cisstMultiTask/mtsTransformationTypes.h>

#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>


#include <sawTrajectories/osaTrajectory.h>
#include <sawTrajectories/sawTrajectoriesExport.h>

class CISST_EXPORT mtsTrajectory : public mtsTaskPeriodic{

private:

    osaTrajectory* trajectory;


    mtsInterfaceProvided* ctl;
    mtsBool mtsEnabled;

    mtsInterfaceProvided* input;

    mtsInterfaceProvided*    output;
    prmPositionJointGet        qout;
    prmPositionCartesianGet   Rtout;


    double t;

protected:

    bool IsEnabled() const { return mtsEnabled; }

    void SetPositionCartesian( const mtsFrm4x4& Rt );
    void SetPositionJoint( const mtsDoubleVec& q );

public:

    mtsTrajectory( const std::string& taskname,
                   double period,
                   const std::string& robfilename,
                   const vctFrame4x4<double>& Rtw0,
                   const vctDynamicVector<double>& qinit );

    ~mtsTrajectory();

    void Configure( const std::string& argv="" );

    void Startup();
    void Run();
    void Cleanup();

};

#endif
