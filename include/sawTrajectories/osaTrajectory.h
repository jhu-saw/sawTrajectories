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


#ifndef _osaTrajectory_h
#define _osaTrajectory_h

#include <cisstRobot/robFunction.h>
#include <cisstRobot/robManipulator.h>
#include <cisstVector/vctFrame4x4.h>
#include <cisstVector/vctDynamicVectorTypes.h>
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
    class IKSegment : public Segment {

    private:

        vctFrame4x4<double> Rtstart; /**< start Cartesian position */
        vctFrame4x4<double> Rtfinal; /**< final Carteisan position */

        double v; /**< linear velocity */
        double w; /**< angular velocity */

    public:

        IKSegment( const vctFrame4x4<double>& Rts,
                   const vctFrame4x4<double>& Rtf,
                   double v, double w);

        //! Get segment start position (Cartesian pos 4x4 matrix)
        vctFrame4x4<double> StateStart() const { return Rtstart; }
        //! Get segment final position (Cartesian pos 4x4 matrix)
        vctFrame4x4<double> StateFinal() const { return Rtfinal; }

        //! Get segment linear velocity
        double VelocityLinear() const { return v; }
        //! Get segment angular velocity
        double VelocityAngular() const { return w; }

    }; // IK segment


    // ZC: NOT USED ?
    // Forward kinematics segment
    class FKSegment : public Segment {

    private:

        vctDynamicVector<double> qstart;
        vctDynamicVector<double> qfinal;

    public:

        FKSegment( const vctDynamicVector<double>& qs,
                   const vctDynamicVector<double>& qf,
                   double ) :
            Segment( osaTrajectory::SE3, osaTrajectory::RN ),
            qstart( qs ),
            qfinal( qf ){}

        //! Get segment start position (joint space ??? ZC)
        vctDynamicVector<double> StateStart() const { return qstart; }
        //! Get segment final position (joint space ??? ZC)
        vctDynamicVector<double> StateFinal() const { return qfinal; }

    }; // FK segment


    // Joint segment
    class RnSegment : public Segment {

    private:

        vctDynamicVector<double> qstart; /**< start joint position */
        vctDynamicVector<double> qfinal; /**< final joint position */

        vctDoubleVec vel; /**< joint velocity */

    public:

        RnSegment( const vctDynamicVector<double>& qs,
                   const vctDynamicVector<double>& qf,
                   double );

        //! Get segment start position (joint space ??? ZC)
        vctDynamicVector<double> StateStart() const { return qstart; }
        //! Get segment final position (joint space ??? ZC)
        vctDynamicVector<double> StateFinal() const { return qfinal; }

        //! Get segment linear velocity
        vctDoubleVec VelocityJoint() const { return vel; }

    }; // Joint segment


    double GetTime() const { return currenttime; }
    void   SetTime( double t ){ currenttime = t; }

    osaTrajectory::Errno PostProcess();

public:

    osaTrajectory( const std::string& robotfilename,
                   const vctFrame4x4<double>& Rtw0,
                   const vctDynamicVector<double>& qinit );

    /**
     * @brief Insert inverse kinematic segment
     *
     * @param Rt2
     * @param vmax
     * @param wmax
     * @return osaTrajectory::Errno   Error number
     */
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
