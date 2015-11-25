
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_PrecLand_IRLock.h>

extern const AP_HAL::HAL& hal;

// Constructor
AC_PrecLand_IRLock::AC_PrecLand_IRLock(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
    : AC_PrecLand_Backend(frontend, state),
      irlock()
{
}

// init - perform initialisation of this backend
void AC_PrecLand_IRLock::init()
{
    irlock.init();

    // set healthy
    _state.healthy = irlock.healthy();
}

// update - give chance to driver to get updates from sensor
bool AC_PrecLand_IRLock::update()
{
    // get new sensor data
    return (irlock.update());
}

// get_angle_to_target - returns body frame angles (in radians) to target
//  returns true if angles are available, false if not (i.e. no target)
//  x_angle_rad : body-frame roll direction, positive = target is to right (looking down)
//  y_angle_rad : body-frame pitch direction, postiive = target is forward (looking down)
bool AC_PrecLand_IRLock::get_angle_to_target(float &x_angle_rad, float &y_angle_rad) const
{
    return irlock.get_angle_to_target(x_angle_rad, y_angle_rad);
}

// get_angles_to_targets - return body frame angles (in radians) to two targets
// returns true if the number of detected targets are exactly two, only then target detection can be reliable
//  x_angle_rad_1 : body-frame roll direction, positive = target is to right (looking down)
//  y_angle_rad_1 : body-frame pitch direction, positive = target is forward (looking down)
//  x_angle_rad_2 : body-frame roll direction, positive = target is to right (looking down)
//  y_angle_rad_2 : body-frame pitch direction, positive = target is forward (looking down)
bool AC_PrecLand_IRLock::get_angles_to_targets(float &x_angle_rad_1, float &y_angle_rad_1,
        float &x_angle_rad_2, float &y_angle_rad_2) const
{
    return irlock.get_angles_to_targets(x_angle_rad_1, y_angle_rad_1, x_angle_rad_2, y_angle_rad_2);
}

//get number of targets
void AC_PrecLand_IRLock::getNumOfTargets( uint8_t& numOfTargets ) const
{
    irlock.getNumOfTargets(numOfTargets);
}

