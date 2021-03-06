/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AC_PRECLAND_BACKEND_H__
#define __AC_PRECLAND_BACKEND_H__

#include <AP_Common.h>
#include <AP_Math.h>
#include <AC_PID.h>             // PID library
#include <AP_InertialNav.h>     // Inertial Navigation library
#include <AC_PrecLand.h>        // Precision Landing frontend

class AC_PrecLand_Backend
{
public:

    // Constructor
    AC_PrecLand_Backend(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state) :
        _frontend(frontend),
        _state(state) {}

    // destructor
    virtual ~AC_PrecLand_Backend() {}

    // init - perform any required initialisation of backend controller
    virtual void init() = 0;

    // update - give chance to driver to get updates from sensor
    //  returns true if new data available
    virtual bool update() = 0;

    // get_angle_to_target - returns body frame angles (in radians) to target
    //  returns true if angles are available, false if not (i.e. no target)
    //  x_angle_rad : body-frame roll direction, positive = target is to right (looking down)
    //  y_angle_rad : body-frame pitch direction, postiive = target is forward (looking down)
    virtual bool get_angle_to_target(float &x_angle_rad, float &y_angle_rad) const = 0;

    // get_angles_to_targets - return body frame angles (in radians) to two targets
    // returns true if the number of detected targets are exactly two, only then target detection can be reliable
    //  x_angle_rad_1 : body-frame roll direction, positive = target is to right (looking down)
    //  y_angle_rad_1 : body-frame pitch direction, positive = target is forward (looking down)
    //  x_angle_rad_2 : body-frame roll direction, positive = target is to right (looking down)
    //  y_angle_rad_2 : body-frame pitch direction, positive = target is forward (looking down)
    virtual bool get_angles_to_targets(float &x_angle_rad_1, float &y_angle_rad_1,
            float &x_angle_rad_2, float &y_angle_rad_2) const = 0;

    //get number of targets
    virtual void getNumOfTargets( uint8_t& numOfTargets ) const = 0;

protected:

    const AC_PrecLand&  _frontend;          // reference to precision landing front end
    AC_PrecLand::precland_state &_state;    // reference to this instances state
};
#endif	// __AC_PRECLAND_BACKEND_H__
