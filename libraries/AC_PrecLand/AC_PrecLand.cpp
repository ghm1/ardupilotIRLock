/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_PrecLand.h>
#include <AC_PrecLand_Backend.h>
#include <AC_PrecLand_Companion.h>
#include <AC_PrecLand_IRLock.h>
#include <AP_Math.h>
#include <AP_Notify.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_PrecLand::var_info[] PROGMEM = {
    // @DisplayName: Precision Land enabled/disabled and behaviour
    // @Description: Precision Land enabled/disabled and behaviour
    // @Values: 0:Disabled, 1:Enabled Always Land, 2:Enabled Strict
    // @User: Advanced
    AP_GROUPINFO("ENABLED", 0, AC_PrecLand, _enabled, 0),

    // @Param: TYPE
    // @DisplayName: Precision Land Type
    // @Description: Precision Land Type
    // @Values: 0:None, 1:CompanionComputer, 2:IRLock
    // @User: Advanced
    AP_GROUPINFO("TYPE",    1, AC_PrecLand, _type, 0),

    // @Param: SPEED
    // @DisplayName: Precision Land horizontal speed maximum in cm/s
    // @Description: Precision Land horizontal speed maximum in cm/s
    // @Range: 0 500
    // @User: Advanced
    AP_GROUPINFO("SPEED",   2, AC_PrecLand, _speed_xy, AC_PRECLAND_SPEED_XY_DEFAULT),

    // @Param: DEBUG_OUTPUT
    // @DisplayName: Precision Land horizontal speed maximum in cm/s
    // @Description: Precision Land horizontal speed maximum in cm/s
    // @Values: 0:None, 1: alarm tone, 2:alarm tone and console output, 3:  console output
    // @User: Advanced
    //AP_GROUPINFO("DEBUG_OUTPUT",   3, AC_PrecLand, _debug_output, 0),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PrecLand::AC_PrecLand(const AP_AHRS& ahrs, const AP_InertialNav& inav) :
        // TMS AC_PI_2D& pi_precland_xy, float dt) :
    _ahrs(ahrs),
    _inav(inav),
    //_pi_precland_xy(pi_precland_xy),
    //_dt(dt),
    _have_estimate(false),
    _backend(NULL),
    _distance_degrees(0.0)
{
    // set parameters to defaults
    AP_Param::setup_object_defaults(this, var_info);

    // other initialisation
    _backend_state.healthy = false;

    //ghm1: initialize correctly
    //_enabled = 1;
    //_type = 2;
    //better: preclnd_enabled = 1 and preclnd_type = 2
    _debug_output = 1;

    _measure_image_distance = true;
    _image_distance_estimate = false;

    _numOfTargets = 0;
    _distanceOfTargetsRad = 0.0;
    _altitudeAboveTarget = 0.0;
}


// init - perform any required initialisation of backends
void AC_PrecLand::init()
{
    // exit immediately if init has already been run
    if (_backend != NULL) {
        return;
    }

    // default health to false
    _backend = NULL;
    _backend_state.healthy = false;

    // instantiate backend based on type parameter
    switch ((enum PrecLandType)(_type.get())) {
        // no type defined
        case PRECLAND_TYPE_NONE:
        default:
            return;
        // companion computer
        case PRECLAND_TYPE_COMPANION:
            _backend = new AC_PrecLand_Companion(*this, _backend_state);
            break;
        // IR Lock
        case PRECLAND_TYPE_IRLOCK:
            _backend = new AC_PrecLand_IRLock(*this, _backend_state);
            break;
    }

    // init backend
    if (_backend != NULL) {
        _backend->init();
    }
}

// update - give chance to driver to get updates from sensor
void AC_PrecLand::update(float alt_above_terrain_cm)
{
    // run backend update
    if (_backend != NULL) {
        // read from sensor
        _backend->update();

        // calculate angles to target and position estimate
        calc_angles_and_pos(alt_above_terrain_cm);
    }
}

// get_target_shift - returns 3D vector of earth-frame position adjustments to target
Vector3f AC_PrecLand::get_target_shift(const Vector3f &orig_target)
{
    Vector3f shift; // default shift initialised to zero

    // do not shift target if not enabled or no position estimate
    if (_backend == NULL || !_have_estimate) {
        return shift;
    }

    // shift is target_offset - (original target - current position)
    Vector3f curr_offset_from_target = orig_target - _inav.get_position();
    shift = _target_pos_offset - curr_offset_from_target;
    shift.z = 0.0f;

    // record we have consumed this reading (perhaps there is a cleaner way to do this using timestamps)
    _have_estimate = false;

    // return adjusted target
    return shift;
}

int first = 1;
uint32_t oldtime = 0;

// calc_angles_and_pos - converts sensor's body-frame angles to earth-frame angles and position estimate
//  body-frame angles stored in _bf_angle_to_target
//  earth-frame angles stored in _ef_angle_to_target
//  position estimate is stored in _target_pos
void AC_PrecLand::calc_angles_and_pos(float alt_above_terrain_cm)
{
    // exit immediately if not enabled
    if (_backend == NULL) {
        _have_estimate = false;
        return;
    }

    // get body-frame angles to target from backend
    if (!_backend->get_angle_to_target(_bf_angle_to_target.x, _bf_angle_to_target.y)) {
        _have_estimate = false;

        //in this case reset tone alarm
        //AP_Notify::flags.target_in_cam_fov = 0;
        _numOfTargets = 0;
        _distanceOfTargetsRad = 0.0;
        _altitudeAboveTarget = 0.0;

        return;
    }

    //reset flag
    _image_distance_estimate = false;
    //update num of targets
    _backend->getNumOfTargets(_numOfTargets);


    //We want to detect exactly two targets to measure distance.
    //The target position is then the middle of these two targets.
    if( _measure_image_distance &&
            _backend->get_angles_to_targets(_bf_angle_to_target_1.x, _bf_angle_to_target_1.y,
            _bf_angle_to_target_2.x, _bf_angle_to_target_2.y))
    {
         float xWidth = fabs(_bf_angle_to_target_1.x - _bf_angle_to_target_2.x);
         float yWidth = fabs(_bf_angle_to_target_1.y - _bf_angle_to_target_2.y);
         //pythagoras an dieser stelle möglich, da es dem pytagors der pixelwerte entspricht, nur, dass kalibrierwert_pixel_per_rad
         //bereits vorher verrechnet wurde
         _distanceOfTargetsRad = pythagorous2(xWidth, yWidth);
         _altitudeAboveTarget = TARGET_SIZE_M / tanf(_distanceOfTargetsRad);
         _image_distance_estimate = true;

         //find midpoint between two targets
         //Vector2f midPtRad = (_bf_angle_to_target_1 + _bf_angle_to_target_2) / 2;
    }
    else
    {
        _distanceOfTargetsRad = 0.0;
        _altitudeAboveTarget = 0.0;
    }

    // subtract vehicle lean angles
    float x_rad = _bf_angle_to_target.x - _ahrs.roll;
    float y_rad = -_bf_angle_to_target.y + _ahrs.pitch;

    //check if we have to output an alarmtone and calculate euclidean distance
    /*if( _debug_output && _debug_output < 3 )
    {
        if(_have_estimate)
        {
            _distance_degrees = abs(degrees(pythagorous2(x_rad,y_rad)));
            if(_distance_degrees < 10.0)
            {
                AP_Notify::flags.target_in_cam_fov = 1;
            }
            else if(_distance_degrees < 20.0)
            {
                AP_Notify::flags.target_in_cam_fov = 2;
            }
            else if(_distance_degrees < 30.0)
            {
                AP_Notify::flags.target_in_cam_fov = 3;
            }
            else if(_distance_degrees < 40.0)
            {
                AP_Notify::flags.target_in_cam_fov = 4;
            }
            else
            {
                AP_Notify::flags.target_in_cam_fov = 0;
            }
        }
        else
        {
            AP_Notify::flags.target_in_cam_fov = 0;
        }
    }*/

    //ghm1 debug
    _bf_offset_to_target.x = alt_above_terrain_cm * tanf(x_rad);
    _bf_offset_to_target.y = alt_above_terrain_cm * tanf(y_rad);
    //float bf_target_pos_offset_x = alt_above_terrain_cm * tanf(x_rad);
    //float bf_target_pos_offset_y = alt_above_terrain_cm * tanf(y_rad);

    // rotate to earth-frame angles
    _ef_angle_to_target.x = y_rad*_ahrs.cos_yaw() - x_rad*_ahrs.sin_yaw();
    _ef_angle_to_target.y = y_rad*_ahrs.sin_yaw() + x_rad*_ahrs.cos_yaw();

    // get current altitude (constrained to no lower than 50cm)
    float alt = max(alt_above_terrain_cm, 50.0f);
    //float alt = alt_above_terrain_cm;

    // convert earth-frame angles to earth-frame position offset

    _target_pos_offset.x = alt*tanf(_ef_angle_to_target.x);
    _target_pos_offset.y = alt*tanf(_ef_angle_to_target.y);
    _target_pos_offset.z = 0;  // not used

    //output to console if activated by parameter settings
    if(_debug_output > 1)
    {
        if(first){
            oldtime = hal.scheduler->millis();
            first = 0;
        }
        uint32_t tnow = hal.scheduler->millis();
        //message every second
        uint32_t diff = tnow - oldtime;
        if( diff > 1000 )
        {
            oldtime = tnow;

            /*if(_have_estimate)
            {
                hal.console->printf("estimate correct\n");
            }
            else
            {
                hal.console->printf("no estimate\n");
            }*/

            //hal.console->printf("target_in_cam_fov: %u\n", AP_Notify::flags.target_in_cam_fov);
            //hal.console->printf("roll: %f\n", _ahrs.roll);
            //hal.console->printf("pitch: %f\n", _ahrs.pitch);
            //hal.console->printf("distance_degrees: %f\n", _distance_degrees);
            //hal.console->printf("bf_angle_to_target_x: %f\n", _bf_angle_to_target.x);
            //hal.console->printf("bf_angle_to_target_y: %f\n", _bf_angle_to_target.y);
            //hal.console->printf("ef_angle_to_target_x: %f\n", _ef_angle_to_target.x);
            //hal.console->printf("ef_angle_to_target_y: %f\n", _ef_angle_to_target.y);
            //hal.console->printf("alt_above_terrain_cm: %f\n", alt_above_terrain_cm);
            //hal.console->printf("target_pos_offset_x: %f\n", _target_pos_offset.x);
            //hal.console->printf("target_pos_offset_y: %f\n", _target_pos_offset.y);

            if(_image_distance_estimate)
            {
                hal.console->printf("widthRad: %f\n", _distanceOfTargetsRad);
                hal.console->printf("altitudeAboveTarget: %f\n", _altitudeAboveTarget);
                hal.console->printf("altitudeSonar %f\n", alt_above_terrain_cm);
            }
            else
            {
                hal.console->printf("no image_distance_estimate\n");
                hal.console->printf("numOfTargets: %u\n", _numOfTargets);
            }

            hal.console->printf("\n");
        }
    }

    //reset have_estimate
    _have_estimate = true;
}
