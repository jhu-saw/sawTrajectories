#include <sawKeyboard/mtsKeyboard.h>
#include <sawTrajectories/mtsTrajectory.h>

#include <cisstCommon/cmnPath.h>

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsTaskManager.h>

//#include <cisstParameterTypes/prmPositionJointGet.h>
//#include <cisstParameterTypes/prmForceTorqueJointSet.h>

// #include <native/task.h>
// #include <sys/mman.h>

class SetPoints : public mtsTaskPeriodic {

private:

  robManipulator* manipulator;

  mtsFunctionWrite SetPositionCartesian;

  vctFrame4x4<double> Rt;
  double x;
  vctDynamicVector<double> q;
  double t;

public:

  SetPoints( const std::string& robotfilename,
	     const vctFrame4x4<double>& Rtw0,
	     const vctDynamicVector<double>& qinit ) :
    mtsTaskPeriodic( "setpoint", 0.1, true ),
    manipulator( NULL ),
    q( qinit ),
    t( 0.0 ){

    manipulator = new robManipulator( robotfilename, Rtw0 );

    Rt = manipulator->ForwardKinematics( q );
    x = Rt[0][3];

    mtsInterfaceRequired* output = AddInterfaceRequired( "Output" );
    if( output ){
      output->AddFunction( "SetPositionCartesian", SetPositionCartesian );
    }
    else{
      CMN_LOG_RUN_ERROR << "Failed to create interface Output for " << GetName()
			<< std::endl;
    }

  }

  void Configure( const std::string& ){}
  void Startup(){}
  void Run(){
    ProcessQueuedCommands();

    t += GetPeriodicity();
    double dx = 0.1 * sin( t - cmnPI_2 ) + 0.1;
    Rt[0][3] = x + -dx;

    mtsFrm4x4 mtsRt( Rt );
    SetPositionCartesian( mtsRt );

  }
  void Cleanup(){}

};



int main( int, char** ){

    /*
  mlockall(MCL_CURRENT | MCL_FUTURE);
  RT_TASK task;
  rt_task_shadow( &task, "mtsTrajectoryExample", 20, 0 );
    */
  mtsTaskManager* taskManager = mtsTaskManager::GetInstance();

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  mtsKeyboard kb;
  kb.SetQuitKey( 'q' );
  kb.AddKeyWriteFunction( 'E', "Enable", "Enable", true );
  taskManager->AddComponent( &kb );

  vctMatrixRotation3<double> Rw0(  0.0,  0.0, -1.0,
                                   0.0,  1.0,  0.0,
                                   1.0,  0.0,  0.0 );
  vctFixedSizeVector<double,3> tw0(0.0);
  vctFrame4x4<double> Rtw0( Rw0, tw0 );


  vctDynamicVector<double> qinit( 7, 0.0 );
  qinit[1] = -cmnPI_2;
  qinit[3] =  cmnPI;
  qinit[5] = -cmnPI_2;

  cmnPath path;
  path.AddRelativeToCisstShare("/models/WAM");
  std::string fname = path.Find("wam7.rob");
  mtsTrajectory trajectory( "trajectory",
			    0.01,
			    fname,
			    Rtw0,
			    qinit );
  taskManager->AddComponent( &trajectory );

  SetPoints setpoints( fname, Rtw0, qinit );
  taskManager->AddComponent( &setpoints );


  taskManager->Connect( trajectory.GetName(), "Control",
			kb.GetName(),         "Enable" );

  taskManager->Connect( trajectory.GetName(), "Input",
			setpoints.GetName(),  "Output" );

  taskManager->CreateAll();
  taskManager->StartAll();

  pause();

  taskManager->KillAll();
  taskManager->Cleanup();



  return 0;

}
