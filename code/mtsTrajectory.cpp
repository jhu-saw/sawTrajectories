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
    if( trajectory->Insert( vctRt, 0.1 ) != osaTrajectory::ESUCCESS ){
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

      vctFrm3 frm3(static_cast<vctRot3>(Rt.Rotation()),Rt.Translation()); // This breaks without the static cast. Blame cisstParameterTypes.
      Rtout.SetPosition(frm3);

      qout.SetPosition(q);

    }
    
    else
      { CMN_LOG_RUN_ERROR << "No trajectory" << std::endl; }

  }


}

void mtsTrajectory::Cleanup(){}

