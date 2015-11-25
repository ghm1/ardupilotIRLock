/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_PrecLand_Companion.h>

extern const AP_HAL::HAL& hal;

// Constructor
AC_PrecLand_Companion::AC_PrecLand_Companion(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
: AC_PrecLand_Backend(frontend, state)
{
}

// init - perform initialisation of this backend
void AC_PrecLand_Companion::init()
{
    // set healthy
    _state.healthy = true;
}

// update - give chance to driver to get updates from sensor
//  returns true if new data available
bool AC_PrecLand_Companion::update()
{
    // To-Do: read target position from companion computer via MAVLink
    return false;
}

// get_angle_to_target - returns body frame angles (in radians) to target
//  returns true if angles are available, false if not (i.e. no target)
//  x_angle_rad : body-frame roll direction, positive = target is to right (looking down)
//  y_angle_rad : body-frame pitch direction, postiive = target is forward (looking down)
bool AC_PrecLand_Companion::get_angle_to_target(float &x_angle_rad, float &y_angle_rad) const
{
    return false;
}

// get_angles_to_targets - retrieve body frame x and y angles (in radians) to targets
//  returns true if angles are available and there are exactly two targets, false if not (i.e. no target)
bool AC_PrecLand_Companion::get_angles_to_targets(float &x_angle_rad_1, float &y_angle_rad_1,
        float &x_angle_rad_2, float &y_angle_rad_2) const
{
    return false;
}

//get number of targets
void AC_PrecLand_Companion::getNumOfTargets( uint8_t& numOfTargets ) const
{
    numOfTargets = 0;
}

