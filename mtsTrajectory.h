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
