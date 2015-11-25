
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AC_PRECLAND_IRLOCK_H__
#define __AC_PRECLAND_IRLOCK_H__

#include <AP_Common.h>
#include <AP_Math.h>
#include <AC_PrecLand_Backend.h>    // Precision Landing backend
#include <AP_IRLock.h>

/*
 * AC_PrecLand_IRLock - implements precision landing using target vectors provided
 *                         by a companion computer (i.e. Odroid) communicating via MAVLink
 */

class AC_PrecLand_IRLock : public AC_PrecLand_Backend
{
public:

    // Constructor
    AC_PrecLand_IRLock(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state);

    // init - perform any required initialisation of backend controller
    void init();

    // update - give chance to driver to get updates from sensor
    //  returns true if new data available
    bool update();

    // get_angle_to_target - returns body frame angles (in radians) to target
    //  returns true if angles are available, false if not (i.e. no target)
    //  x_angle_rad : body-frame roll direction, positive = target is to right (looking down)
    //  y_angle_rad : body-frame pitch direction, postiive = target is forward (looking down)
    bool get_angle_to_target(float &x_angle_rad, float &y_angle_rad) const;

    // get_angles_to_targets - return body frame angles (in radians) to two targets
    // returns true if the number of detected targets are exactly two, only then target detection can be reliable
    //  x_angle_rad_1 : body-frame roll direction, positive = target is to right (looking down)
    //  y_angle_rad_1 : body-frame pitch direction, positive = target is forward (looking down)
    //  x_angle_rad_2 : body-frame roll direction, positive = target is to right (looking down)
    //  y_angle_rad_2 : body-frame pitch direction, positive = target is forward (looking down)
    bool get_angles_to_targets(float &x_angle_rad_1, float &y_angle_rad_1,
            float &x_angle_rad_2, float &y_angle_rad_2) const;

    //get number of targets
    void getNumOfTargets( uint8_t& numOfTargets ) const;

private:
    AP_IRLock_PX4 irlock;

};
#endif	// __AC_PRECLAND_IRLOCK_H__
