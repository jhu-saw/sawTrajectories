
#ifndef _osaTrajectory_h
#define _osaTrajectory_h

#include <cisstRobot/robFunction.h>
#include <cisstRobot/robManipulator.h>
#include <cisstVector/vctFrame4x4.h>
#include <list>

#include <sawTrajectories/sawTrajectoriesExport.h>

class CISST_EXPORT osaTrajectory : public robManipulator{

 public:

  enum Errno
  { 
    ESUCCESS,
    EPENDING,
    EEXPIRED,
    EEMPTY,
    EUNSUPPORTED,
    ETRANSITION,
    EINVALID
  };

 private:

  double currenttime;
  vctDynamicVector<double> q;
  robFunction* function;

  enum Space{ R3, RN, SO3, SE3 };

  // segment base class
  class Segment{

  private:

    osaTrajectory::Space inspace;
    osaTrajectory::Space outspace;

  protected:

    double duration;

  public:
    
    Segment( osaTrajectory::Space  is, osaTrajectory::Space os ) : 
      inspace( is ),
      outspace( os ),
      duration( -1.0 ){}
      
    virtual osaTrajectory::Space  InputSpace() const { return inspace; }
    virtual osaTrajectory::Space  OutputSpace() const { return outspace; }

  };


  std::list< osaTrajectory::Segment* > segments;
  osaTrajectory::Errno LoadNextSegment();


  // Inverse kinematics segment
  class InverseKinematics : public Segment {

  private:

    vctFrame4x4<double> Rtstart;
    vctFrame4x4<double> Rtfinal;
    
    double v;
    double w;

  public:
    
    InverseKinematics( const vctFrame4x4<double>& Rts,
		       const vctFrame4x4<double>& Rtf,
		       double v, double w);

    vctFrame4x4<double> StateStart() const { return Rtstart; }
    vctFrame4x4<double> StateFinal() const { return Rtfinal; }

    double VelocityLinear() const { return v; }
    double VelocityAngular() const { return w; }

  };




  // Forward kinematics segment
  class ForwardKinematics : public Segment {

  private:

    vctDynamicVector<double> qstart;
    vctDynamicVector<double> qfinal;

  public:
    
    ForwardKinematics( const vctDynamicVector<double>& qs,
		       const vctDynamicVector<double>& qf,
		       double ) :
      Segment( osaTrajectory::SE3, osaTrajectory::RN ),
      qstart( qs ),
      qfinal( qf ){}

    vctDynamicVector<double> StateStart() const { return qstart; }
    vctDynamicVector<double> StateFinal() const { return qfinal; }

  };






  // Joint segment
  class Rn : public Segment {

  private:

    vctDynamicVector<double> qstart;
    vctDynamicVector<double> qfinal;

  public:
    
    Rn( const vctDynamicVector<double>& qs,
	const vctDynamicVector<double>& qf,
	double ) :
      Segment( osaTrajectory::RN, osaTrajectory::RN ),
      qstart( qs ),
      qfinal( qf ){}

    vctDynamicVector<double> StateStart() const { return qstart; }
    vctDynamicVector<double> StateFinal() const { return qfinal; }

  };


  double GetTime() const { return currenttime; }
  void   SetTime( double t ){ currenttime = t; }

  osaTrajectory::Errno PostProcess();

 public:

  osaTrajectory( const std::string& robotfilename,
		 const vctFrame4x4<double>& Rtw0,
		 const vctDynamicVector<double>& qinit );

  osaTrajectory::Errno InsertIK( const vctFrame4x4<double>& Rt2,
				 double vmax,
				 double wmax );

  osaTrajectory::Errno Insert( const vctFrame4x4<double>& Rt2, double dt );

  osaTrajectory::Errno Evaluate( double t, 
				 vctFrame4x4<double>& Rt,
				 vctDynamicVector<double>& q );

  friend std::ostream& operator<<( std::ostream& os, 
				   const osaTrajectory::Errno& e ){
    switch( e ){
    case osaTrajectory::ESUCCESS:       os << "SUCCESS";      break;
    case osaTrajectory::EPENDING:       os << "EPENDING";     break;
    case osaTrajectory::EEXPIRED:       os << "EEXPIRED";     break;
    case osaTrajectory::EEMPTY:         os << "EEMPTY";       break;
    case osaTrajectory::EUNSUPPORTED:   os << "EUNSUPPORTED"; break;
    case osaTrajectory::ETRANSITION:    os << "ETRANSITION";  break;
    case osaTrajectory::EINVALID:       os << "EINVALID";     break;
    }
    return os;
  }

};

#endif
