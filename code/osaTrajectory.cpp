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


#include <sawTrajectories/osaTrajectory.h>
#include <cisstRobot/robLinearSE3.h>
#include <cisstRobot/robLinearRn.h>
#include <cisstCommon/cmnLogger.h>

osaTrajectory::osaTrajectory( const std::string& robotfilename,
                              const vctFrame4x4<double>& Rtw0,
                              const vctDynamicVector<double>& qinit ):
    robManipulator( robotfilename, Rtw0 ),
    currenttime( 0.0 ),
    q( qinit ),
    function( NULL ){}


// InverseKinematics Segment
osaTrajectory::IKSegment::IKSegment
( const vctFrame4x4<double>& Rts,
  const vctFrame4x4<double>& Rtf,
  double vmax, double wmax) :
    Segment( osaTrajectory::SE3, osaTrajectory::RN ),
    Rtstart( Rts ),
    Rtfinal( Rtf ),
    v( vmax ),
    w( wmax )
{

    vctFrame4x4<double> Rtsw( Rtstart );
    Rtsw.InverseSelf();
    vctFrame4x4<double> Rtsf = Rtsw * Rtfinal;

    vctFixedSizeVector<double,3> tsf = Rtsf.Translation();

    duration  = tsf.Norm() / vmax;           // translation duration

    vctMatrixRotation3<double> Rsf( Rtsf[0][0], Rtsf[0][1], Rtsf[0][2],
                                    Rtsf[1][0], Rtsf[1][1], Rtsf[1][2],
                                    Rtsf[2][0], Rtsf[2][1], Rtsf[2][2],
                                    VCT_NORMALIZE );
    vctAxisAngleRotation3<double> rsf( Rsf );

    if( duration < rsf.Angle() / wmax )
    { duration = rsf.Angle() / wmax; }     // rotation duration

    v = tsf.Norm()  / duration;              // Recompute the linear velocity
    w = rsf.Angle() / duration;              // Recompute the angular velocity

}



osaTrajectory::RnSegment::RnSegment(
        const vctDynamicVector<double> &qs,
        const vctDynamicVector<double> &qf,
        double):
    Segment( osaTrajectory::RN, osaTrajectory::RN ),
    qstart(qs), qfinal(qf)
{

}




// ---------- Insert Segment ---------------------

osaTrajectory::Errno osaTrajectory::InsertIK( const vctFrame4x4<double>& Rtw2,
                                              double vmax,
                                              double wmax ){

    vctFrame4x4<double> Rtw1;

    // insert after the last segment

    // If segments not empty, get/compute Rtw1 from last segment
    if( !segments.empty() ){

        // get the last segment
        osaTrajectory::Segment* segment = segments.back();

        // is it a joint segment
        osaTrajectory::RnSegment* segmentrn = NULL;
        segmentrn = dynamic_cast< osaTrajectory::RnSegment* >( segment );
        if( segmentrn != NULL )
        { Rtw1 = robManipulator::ForwardKinematics( segmentrn->StateFinal() ); }

        // is it a inverse kinematic segment
        osaTrajectory::IKSegment* segmentik = NULL;
        segmentik = dynamic_cast< osaTrajectory::IKSegment* >( segment );
        if( segmentik != NULL )
        { Rtw1 = segmentik->StateFinal(); }

    }

    else{

        // no function
        if( function == NULL ){
            // use the initial joints
            Rtw1 = robManipulator::ForwardKinematics( q );
        }

        // with current function
        else{

            robLinearSE3* se3fn = dynamic_cast<robLinearSE3*>( function );
            if( se3fn == NULL ){
                CMN_LOG_RUN_ERROR << "Expected an SE3 linear function." << std::endl;
                return osaTrajectory::EUNSUPPORTED;
            }

            vctFixedSizeVector<double,6> vw1;
            vctFixedSizeVector<double,6> vdwd1;
            se3fn->FinalState( Rtw1, vw1, vdwd1 );
        }
    }

    osaTrajectory::IKSegment* segment = NULL;
    segment = new osaTrajectory::IKSegment( Rtw1, Rtw2, vmax, wmax );
    segments.push_back( (osaTrajectory::Segment*) segment );

    return osaTrajectory::ESUCCESS;
}

osaTrajectory::Errno osaTrajectory::Insert( const vctFrame4x4<double>& Rtw2,
                                            double dt ){

    vctFrame4x4<double> Rtw1;
    //std::cout << segments.size() << std::endl;

    // insert after the last segment
    if( !segments.empty() ){

        // get the last segment
        osaTrajectory::Segment* segment = segments.back();

        // is it a joint segment
        osaTrajectory::RnSegment* segmentrn = NULL;
        segmentrn = dynamic_cast< osaTrajectory::RnSegment* >( segment );
        if( segmentrn != NULL )
        { Rtw1 = robManipulator::ForwardKinematics( segmentrn->StateFinal() ); }

        osaTrajectory::IKSegment* segmentik = NULL;
        segmentik = dynamic_cast< osaTrajectory::IKSegment* >( segment );
        if( segmentik != NULL )
        { Rtw1 = segmentik->StateFinal(); }

    }

    else{

        // no function
        if( function == NULL ){
            // use the initial joints
            Rtw1 = robManipulator::ForwardKinematics( q );
        }

        // with current function
        else{

            robLinearSE3* se3fn = dynamic_cast<robLinearSE3*>( function );
            if( se3fn == NULL ){
                CMN_LOG_RUN_ERROR << "Expected an SE3 linear function." << std::endl;
                return osaTrajectory::EUNSUPPORTED;
            }

            vctFixedSizeVector<double,6> vw1;
            vctFixedSizeVector<double,6> vdwd1;
            se3fn->FinalState( Rtw1, vw1, vdwd1 );

        }

    }

    vctFrame4x4<double> Rt1w( Rtw1 );
    Rt1w.InverseSelf();
    vctFrame4x4<double> Rt12 = Rt1w * Rtw2;
    vctMatrixRotation3<double> R12( Rt12[0][0], Rt12[0][1], Rt12[0][2],
                                    Rt12[1][0], Rt12[1][1], Rt12[1][2],
                                    Rt12[2][0], Rt12[2][1], Rt12[2][2],
                                    VCT_NORMALIZE );

    vctFixedSizeVector<double,3> t12 = Rt12.Translation();
    vctAxisAngleRotation3<double> r12( R12 );

    double vmax = t12.Norm() / dt;
    double wmax = r12.Angle() / dt;

    osaTrajectory::IKSegment* segment = NULL;
    segment = new osaTrajectory::IKSegment( Rtw1, Rtw2, vmax, wmax );
    segments.push_back( (osaTrajectory::Segment*) segment );

    return osaTrajectory::ESUCCESS;
}

osaTrajectory::Errno osaTrajectory::LoadNextSegment(){

    if( segments.empty() ){
        //CMN_LOG_RUN_VERBOSE << "Trajectory has no motion segment" << std::endl;
        return osaTrajectory::EEMPTY;
    }

    delete function;

    osaTrajectory::Segment* segment = segments.front();

    // is it a joint segment
    osaTrajectory::RnSegment* segmentrn = NULL;
    segmentrn = dynamic_cast< osaTrajectory::RnSegment* >( segment );
    if( segmentrn != NULL ){
        try{
            function = new robLinearRn(segmentrn->StateStart(),
                                       segmentrn->StateFinal(),
                                       segmentrn->VelocityJoint(),
                                       GetTime() );
        }
        catch(...)
        {
            std::cout << "loadnext" << std::endl;
        }
    }

    // is it a ik segment
    osaTrajectory::IKSegment* segmentik = NULL;
    segmentik = dynamic_cast< osaTrajectory::IKSegment* >( segment );
    if( segmentik != NULL ){
        try{
            function = new robLinearSE3( segmentik->StateStart(),
                                         segmentik->StateFinal(),
                                         segmentik->VelocityLinear(),
                                         segmentik->VelocityAngular(),
                                         GetTime() );
        }
        catch(...)
        { std::cout << "loadnext" << std::endl;}
    }

    segments.pop_front();
    delete segment;

    return osaTrajectory::ESUCCESS;
}


osaTrajectory::Errno 
osaTrajectory::Evaluate( double t, 
                         vctFrame4x4<double>& Rtout,
                         vctDynamicVector<double>& qout ){

    // Is there a function?
    if( function == NULL ){

        // Is there a segment queued? Return the last fk( q )
        if( segments.empty() ){
            SetTime( t );
            try{ Rtout = robManipulator::ForwardKinematics( q ); }
            catch(...){ std::cout << "empty fkin" << std::endl; }
            qout = q;
            return osaTrajectory::EEMPTY;
        }

        // Load the next segment
        if( LoadNextSegment() != osaTrajectory::ESUCCESS ){
            CMN_LOG_RUN_ERROR << "Failed to process next segment." << std::endl;
            return osaTrajectory::ETRANSITION;
        }

    }

    SetTime( t );

    // process the current interpolation function
    robLinearSE3* se3fn = NULL;

    se3fn = dynamic_cast< robLinearSE3* > ( function );
    if( se3fn != NULL ){
        vctFixedSizeVector<double,6> vw;
        vctFixedSizeVector<double,6> vdwd;
        try
        { se3fn->Evaluate( GetTime(), Rtout, vw, vdwd ); }
        catch(...)
        { std::cout << "Eval" << std::endl; }

        vctDynamicVector<double> qnew( q );

        robManipulator::Errno err;
        try
        { err = robManipulator::InverseKinematics( qnew, Rtout, 1e-3, 100 ); }
        catch( ... )
        { std::cout << "IKine" << std::endl; }

        if( err == robManipulator::ESUCCESS ){
            q    = qnew;
            qout = qnew;
            return PostProcess();
        }

        else{

            try{ Rtout = robManipulator::ForwardKinematics( q ); }
            catch(...){ std::cout << "old fkine" << std::endl;}
            qout = q;
            CMN_LOG_RUN_ERROR << "Failed to compute inverse kinematics " << std::endl;
            exit(1);
            return osaTrajectory::EINVALID;
        }

    }

    return osaTrajectory::EUNSUPPORTED;

}

osaTrajectory::Errno osaTrajectory::PostProcess(){

    if( function == NULL ){
        CMN_LOG_RUN_ERROR << "No function to process" << std::endl;
        return osaTrajectory::EEMPTY;
    }


    // The function has not started yet
    if( GetTime() < function->StartTime() )
    { return osaTrajectory::EPENDING; }

    // The function has expired
    if( function->StopTime() < GetTime()  ){
        // load the next segment
        return LoadNextSegment();
    }

    return osaTrajectory::ESUCCESS;
}
