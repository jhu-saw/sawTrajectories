#include <sawTrajectories/osaTrajectory.h>
#include <cisstCommon/cmnPath.h>
#include <cisstOSAbstraction/osaSleep.h>

int main( int, char** ){

  cmnLogger::SetMask( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskFunction( CMN_LOG_ALLOW_ALL );
  cmnLogger::SetMaskDefaultLog( CMN_LOG_ALLOW_ALL );

  vctFrame4x4<double> Rtw0;

  vctDynamicVector<double> qinit( 7, 0.0 );
  qinit[1] = -cmnPI_2;
  qinit[3] =  cmnPI;
  qinit[5] = -cmnPI_2;

  cmnPath path;
  path.AddRelativeToCisstShare("/models/WAM");
  std::string fname = path.Find("wam7.rob");

  osaTrajectory trajectory( fname, Rtw0, qinit );

  robManipulator manipulator( fname, Rtw0 );

  vctDynamicVector<double> q( qinit );

  q[1] = -cmnPI / 4.0;
  q[3] =  cmnPI / 2.0;
  vctFrame4x4<double> Rtw1 = manipulator.ForwardKinematics( q );
  if( trajectory.InsertIK( Rtw1, 0.1, 0.1 ) != osaTrajectory::ESUCCESS ){
    CMN_LOG_RUN_ERROR << "Failed to insert Rtw1" << std::endl;
    return -1;
  }
  vctQuaternionRotation3<double> q1( Rtw1.Rotation() );
  std::cerr << "q1: " << q1 << Rtw1.Translation() << std::endl;

  q[0] = -cmnPI / 4.0;
  q[2] =  cmnPI / 4.0;
  q[6] = -cmnPI / 4.0;
  vctFrame4x4<double> Rtw2 = manipulator.ForwardKinematics( q );
  vctQuaternionRotation3<double> q2( Rtw2.Rotation() );
  trajectory.InsertIK( Rtw2, 0.1, 0.1 );
  std::cerr << "q2: " << q2 << Rtw2.Translation() << std::endl;

  q = qinit;
  vctFrame4x4<double> Rtw3 = manipulator.ForwardKinematics( q );
  vctQuaternionRotation3<double> q3( Rtw3.Rotation() );
  trajectory.InsertIK( Rtw3, 0.1, 0.1 );
  std::cerr << "q3: " << q3 << Rtw3.Translation() << std::endl;

  double t=-1.0;
  while( t<40 ){
    vctFrame4x4<double> Rtwt;
    vctDynamicVector<double> qs;
    osaTrajectory::Errno e = trajectory.Evaluate( t, Rtwt, qs  );
    if( e == osaTrajectory::EEMPTY ){
      if( trajectory.InsertIK( Rtw1, 0.1, 0.1 ) != osaTrajectory::ESUCCESS ){
	CMN_LOG_RUN_ERROR << "Failed to insert Rtw1" << std::endl;
	return -1;
      }
    }

    vctQuaternionRotation3<double> q( Rtwt.Rotation() );
    std::cout << t << " " << q << Rtwt.Translation() << std::endl;
    t += 0.01;
  }


  return 0;

}
