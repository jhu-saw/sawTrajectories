#include <sawTrajectories/osaTrajectory.h>
#include <cisstRobot/robLinearSE3.h>
#include <cisstCommon/cmnLogger.h>

osaTrajectory::osaTrajectory( const std::string& robotfilename,
			      const vctFrame4x4<double>& Rtw0,
			      const vctDynamicVector<double>& qinit ):
  robManipulator( robotfilename, Rtw0 ),
  currenttime( 0.0 ),
  q( qinit ),
  function( NULL ){}

osaTrajectory::InverseKinematics::InverseKinematics
( const vctFrame4x4<double>& Rts,
  const vctFrame4x4<double>& Rtf,
  double vmax, double wmax) :
  Segment( osaTrajectory::SE3, osaTrajectory::RN ),
  Rtstart( Rts ),
  Rtfinal( Rtf ),
  v( vmax ),
  w( wmax ){

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

osaTrajectory::Errno osaTrajectory::InsertIK( const vctFrame4x4<double>& Rtw2,
					      double vmax, 
					      double wmax ){

  vctFrame4x4<double> Rtw1;

  // insert after the last segment
  if( !segments.empty() ){

    // get the last segment
    osaTrajectory::Segment* segment = segments.back();

    // is it a joint segment
    osaTrajectory::Rn* segmentrn = NULL;
    segmentrn = dynamic_cast< osaTrajectory::Rn* >( segment );
    if( segmentrn != NULL )
      { Rtw1 = robManipulator::ForwardKinematics( segmentrn->StateFinal() ); }

    osaTrajectory::InverseKinematics* segmentik = NULL;
    segmentik = dynamic_cast< osaTrajectory::InverseKinematics* >( segment );
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

  osaTrajectory::InverseKinematics* segment = NULL;
  segment = new osaTrajectory::InverseKinematics( Rtw1, Rtw2, vmax, wmax );
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
    osaTrajectory::Rn* segmentrn = NULL;
    segmentrn = dynamic_cast< osaTrajectory::Rn* >( segment );
    if( segmentrn != NULL )
      { Rtw1 = robManipulator::ForwardKinematics( segmentrn->StateFinal() ); }

    osaTrajectory::InverseKinematics* segmentik = NULL;
    segmentik = dynamic_cast< osaTrajectory::InverseKinematics* >( segment );
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

  osaTrajectory::InverseKinematics* segment = NULL;
  segment = new osaTrajectory::InverseKinematics( Rtw1, Rtw2, vmax, wmax );
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
  osaTrajectory::Rn* segmentrn = NULL;
  segmentrn = dynamic_cast< osaTrajectory::Rn* >( segment );
  if( segmentrn != NULL ){}
  
  // is it a ik segment
  osaTrajectory::InverseKinematics* segmentik = NULL;
  segmentik = dynamic_cast< osaTrajectory::InverseKinematics* >( segment );
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
