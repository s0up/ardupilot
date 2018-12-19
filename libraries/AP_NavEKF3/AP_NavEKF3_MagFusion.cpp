#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Control reset of yaw and magnetic field states
void NavEKF3_core::controlMagYawReset()
{

}

// this function is used to do a forced re-alignment of the yaw angle to align with the horizontal velocity
// vector from GPS. It is used to align the yaw angle after launch or takeoff.
void NavEKF3_core::realignYawGPS()
{

}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of magnetometer data
void NavEKF3_core::SelectMagFusion()
{

}

/*
 * Fuse magnetometer measurements using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
*/
void NavEKF3_core::FuseMagnetometer()
{

}


/*
 * Fuse magnetic heading measurement using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
 * This fusion method only modifies the orientation, does not require use of the magnetic field states and is computationally cheaper.
 * It is suitable for use when the external magnetic field environment is disturbed (eg close to metal structures, on ground).
 * It is not as robust to magnetometer failures.
 * It is not suitable for operation where the horizontal magnetic field strength is weak (within 30 degrees latitude of the the magnetic poles)
*/
void NavEKF3_core::fuseEulerYaw()
{

}

/*
 * Fuse declination angle using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
 * This is used to prevent the declination of the EKF earth field states from drifting during operation without GPS
 * or some other absolute position or velocity reference
*/
void NavEKF3_core::FuseDeclination(float declErr)
{
 
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

// align the NE earth magnetic field states with the published declination
void NavEKF3_core::alignMagStateDeclination()
{

}

// record a magnetic field state reset event
void NavEKF3_core::recordMagReset()
{

}


#endif // HAL_CPU_CLASS
