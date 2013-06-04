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

#include <sawTrajectories/mtsTrajectory.h>
#include <cisstVector/vctFrame4x4.h>
#include <cisstOSAbstraction/osaCPUAffinity.h>

mtsTrajectory::mtsTrajectory( const std::string& taskname,
                              double period,
                              const std::string& robotfilename,
                              const vctFrame4x4<double>& Rtw0,
                              const vctDynamicVector<double>& qinit ) :
    mtsTaskPeriodic( taskname, period, true ),
    trajectory( NULL ),
    ctl( NULL ),
    mtsEnabled( false ),
    input( NULL ),
    output( NULL ),
    t( 0.0 ){

    trajectory = new osaTrajectory( robotfilename, Rtw0, qinit );

    // The control interface: to change the state of the component
    ctl = AddInterfaceProvided( "Control" );
    if( ctl ){

        StateTable.AddData( mtsEnabled, "Enabled" );
        ctl->AddCommandWriteState( StateTable, mtsEnabled, "Enable" );

    }
    else{
        CMN_LOG_RUN_ERROR << "Failed to create interface Control for " << GetName()
                          << std::endl;
    }


    // Input interface: to add positions in the queue
    input = AddInterfaceProvided( "Input" );
    if( input ){

        input->AddCommandWrite( &mtsTrajectory::SetPositionCartesian,
                                this,
                                "SetPositionCartesian" );
        input->AddCommandWrite( &mtsTrajectory::SetPositionJoint,
                                this,
                                "SetPositionJoint" );

    }
    else{
        CMN_LOG_RUN_ERROR << "Failed to create interface Input for " << GetName()
                          << std::endl;
    }


    // Output interface: to get the next desired robot state
    output = AddInterfaceProvided( "Output" );
    if( output ){

        StateTable.AddData( Rtout, "GetPositionCartesian" );
        output->AddCommandReadState( StateTable, Rtout, "GetPositionCartesian" );

        qout.SetSize( 7 );
        qout.Position().SetAll( 0.0 );

        StateTable.AddData( qout, "GetPositionJoint" );
        output->AddCommandReadState( StateTable, qout, "GetPositionJoint" );

    }
    else{
        CMN_LOG_RUN_ERROR << "Failed to create interface Input for " << GetName()
                          << std::endl;
    }


}

mtsTrajectory::~mtsTrajectory(){}

void mtsTrajectory::SetPositionCartesian( const mtsFrm4x4& mtsRt ){

    if( trajectory ){
        vctMatrixRotation3<double> vctR( mtsRt[0][0], mtsRt[0][1], mtsRt[0][2],
                                         mtsRt[1][0], mtsRt[1][1], mtsRt[1][2],
                                         mtsRt[2][0], mtsRt[2][1], mtsRt[2][2],
                                         VCT_NORMALIZE );
        vctFrame4x4<double> vctRt( vctR, mtsRt.Translation() );
        if( trajectory->Insert( vctRt, 0.01 ) != osaTrajectory::ESUCCESS ){
            CMN_LOG_RUN_ERROR << "Failed to queue Cartesian position" << std::endl;
        }
    }

    else
    { CMN_LOG_RUN_ERROR << "No trajectory" << std::endl; }

}

void mtsTrajectory::SetPositionJoint( const mtsDoubleVec& ){}

void mtsTrajectory::Configure( const std::string& ){}

void mtsTrajectory::Startup(){

    osaCPUSetAffinity( OSA_CPU2 );

}

void mtsTrajectory::Run(){

    ProcessQueuedCommands();

    if( IsEnabled() ){

        if( trajectory ){

            t += GetPeriodicity();

            vctFrame4x4<double> Rt;
            vctDynamicVector<double> q;
            try
            { osaTrajectory::Errno err = trajectory->Evaluate( t, Rt, q ); }
            catch( ... )
            { std::cout << "CAUGHT!" << std::endl; }

            // This breaks without the static cast. Blame cisstParameterTypes.
            vctFrm3 frm3(static_cast<vctRot3>(Rt.Rotation()),Rt.Translation());
            Rtout.SetPosition(frm3);

            qout.SetPosition(q);
            //std::cout << q << std::endl;
        }

        else
        { CMN_LOG_RUN_ERROR << "No trajectory" << std::endl; }

    }


}

void mtsTrajectory::Cleanup(){}

