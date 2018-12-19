#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define earthRate 0.000072921f // earth rotation rate (rad/sec)

// initial imu bias uncertainty (deg/sec)
#define INIT_ACCEL_BIAS_UNCERTAINTY 0.5f

// maximum allowed gyro bias (rad/sec)
#define GYRO_BIAS_LIMIT 0.5f

// constructor
NavEKF2_core::NavEKF2_core(void) :
    _perf_UpdateFilter(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_UpdateFilter")),
    _perf_CovariancePrediction(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_CovariancePrediction")),
    _perf_FuseVelPosNED(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseVelPosNED")),
    _perf_FuseMagnetometer(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseMagnetometer")),
    _perf_FuseAirspeed(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseAirspeed")),
    _perf_FuseSideslip(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseSideslip")),
    _perf_TerrainOffset(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_TerrainOffset")),
    _perf_FuseOptFlow(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseOptFlow"))
{
    _perf_test[0] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test0");
    _perf_test[1] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test1");
    _perf_test[2] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test2");
    _perf_test[3] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test3");
    _perf_test[4] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test4");
    _perf_test[5] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test5");
    _perf_test[6] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test6");
    _perf_test[7] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test7");
    _perf_test[8] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test8");
    _perf_test[9] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test9");
}

// setup this core backend
bool NavEKF2_core::setup_core(NavEKF2 *_frontend, uint8_t _imu_index, uint8_t _core_index)
{
    frontend = _frontend;
    imu_index = _imu_index;
    core_index = _core_index;
    _ahrs = frontend->_ahrs;

    /*
      the imu_buffer_length needs to cope with a 260ms delay at a
      maximum fusion rate of 100Hz. Non-imu data coming in at faster
      than 100Hz is downsampled. For 50Hz main loop rate we need a
      shorter buffer.
     */
    if (AP::ins().get_sample_rate() < 100) {
        imu_buffer_length = 13;
    } else {
        // maximum 260 msec delay at 100 Hz fusion rate
        imu_buffer_length = 26;
    }
    if(!storedGPS.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedMag.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedBaro.init(OBS_BUFFER_LENGTH)) {
        return false;
    } 
    if(!storedTAS.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedOF.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    // Note: the use of dual range finders potentially doubles the amount of to be stored
    if(!storedRange.init(2*OBS_BUFFER_LENGTH)) {
        return false;
    }
    // Note: range beacon data is read one beacon at a time and can arrive at a high rate
    if(!storedRangeBeacon.init(imu_buffer_length)) {
        return false;
    }
    if(!storedExtNav.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedIMU.init(imu_buffer_length)) {
        return false;
    }
    if(!storedOutput.init(imu_buffer_length)) {
        return false;
    }

    return true;
}
    

/********************************************************
*                   INIT FUNCTIONS                      *
********************************************************/

// Use a function call rather than a constructor to initialise variables because it enables the filter to be re-started in flight if necessary.
void NavEKF2_core::InitialiseVariables()
{

}


/*
  separate out the mag reset so it can be used when compass learning completes
 */
void NavEKF2_core::InitialiseVariablesMag()
{

}

// Initialise the states from accelerometer and magnetometer data (if present)
// This method can only be used when the vehicle is static
bool NavEKF2_core::InitialiseFilterBootstrap(void)
{

    // we initially return false to wait for the IMU buffer to fill
    return false;
}

// initialise the covariance matrix
void NavEKF2_core::CovarianceInit()
{

}

/********************************************************
*                 UPDATE FUNCTIONS                      *
********************************************************/
// Update Filter States - this should be called whenever new IMU data is available
void NavEKF2_core::UpdateFilter(bool predict)
{

}

void NavEKF2_core::correctDeltaAngle(Vector3f &delAng, float delAngDT)
{

}

void NavEKF2_core::correctDeltaVelocity(Vector3f &delVel, float delVelDT)
{

}

/*
 * Update the quaternion, velocity and position states using delayed IMU measurements
 * because the EKF is running on a delayed time horizon. Note that the quaternion is
 * not used by the EKF equations, which instead estimate the error in the attitude of
 * the vehicle when each observtion is fused. This attitude error is then used to correct
 * the quaternion.
*/
void NavEKF2_core::UpdateStrapdownEquationsNED()
{

}

/*
 * Propagate PVA solution forward from the fusion time horizon to the current time horizon
 * using simple observer which performs two functions:
 * 1) Corrects for the delayed time horizon used by the EKF.
 * 2) Applies a LPF to state corrections to prevent 'stepping' in states due to measurement
 * fusion introducing unwanted noise into the control loops.
 * The inspiration for using a complementary filter to correct for time delays in the EKF
 * is based on the work by A Khosravian.
 *
 * "Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements"
 * A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
*/
void NavEKF2_core::calcOutputStates()
{

}

/*
 * Calculate the predicted state covariance matrix using algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
*/
void NavEKF2_core::CovariancePrediction()
{


}

// zero specified range of rows in the state covariance matrix
void NavEKF2_core::zeroRows(Matrix24 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=first; row<=last; row++)
    {
        memset(&covMat[row][0], 0, sizeof(covMat[0][0])*24);
    }
}

// zero specified range of columns in the state covariance matrix
void NavEKF2_core::zeroCols(Matrix24 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=0; row<=23; row++)
    {
        memset(&covMat[row][first], 0, sizeof(covMat[0][0])*(1+last-first));
    }
}

// reset the output data to the current EKF state
void NavEKF2_core::StoreOutputReset()
{
    outputDataNew.quat = stateStruct.quat;
    outputDataNew.velocity = stateStruct.velocity;
    outputDataNew.position = stateStruct.position;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i] = outputDataNew;
    }
    outputDataDelayed = outputDataNew;
    // reset the states for the complementary filter used to provide a vertical position dervative output
    posDown = stateStruct.position.z;
    posDownDerivative = stateStruct.velocity.z;
}

// Reset the stored output quaternion history to current EKF state
void NavEKF2_core::StoreQuatReset()
{
    outputDataNew.quat = stateStruct.quat;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].quat = outputDataNew.quat;
    }
    outputDataDelayed.quat = outputDataNew.quat;
}

// Rotate the stored output quaternion history through a quaternion rotation
void NavEKF2_core::StoreQuatRotate(Quaternion deltaQuat)
{
    outputDataNew.quat = outputDataNew.quat*deltaQuat;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].quat = storedOutput[i].quat*deltaQuat;
    }
    outputDataDelayed.quat = outputDataDelayed.quat*deltaQuat;
}

// calculate nav to body quaternions from body to nav rotation matrix
void NavEKF2_core::quat2Tbn(Matrix3f &Tbn, const Quaternion &quat) const
{
    // Calculate the body to nav cosine matrix
    quat.rotation_matrix(Tbn);
}

// force symmetry on the covariance matrix to prevent ill-conditioning
void NavEKF2_core::ForceSymmetry()
{
    for (uint8_t i=1; i<=stateIndexLim; i++)
    {
        for (uint8_t j=0; j<=i-1; j++)
        {
            float temp = 0.5f*(P[i][j] + P[j][i]);
            P[i][j] = temp;
            P[j][i] = temp;
        }
    }
}

// copy covariances across from covariance prediction calculation
void NavEKF2_core::CopyCovariances()
{
    // copy predicted covariances
    for (uint8_t i=0; i<=stateIndexLim; i++) {
        for (uint8_t j=0; j<=stateIndexLim; j++)
        {
            P[i][j] = nextP[i][j];
        }
    }
}

// constrain variances (diagonal terms) in the state covariance matrix to  prevent ill-conditioning
void NavEKF2_core::ConstrainVariances()
{

}

// constrain states to prevent ill-conditioning
void NavEKF2_core::ConstrainStates()
{

}

// calculate the NED earth spin vector in rad/sec
void NavEKF2_core::calcEarthRateNED(Vector3f &omega, int32_t latitude) const
{
    float lat_rad = radians(latitude*1.0e-7f);
    omega.x  = earthRate*cosf(lat_rad);
    omega.y  = 0;
    omega.z  = -earthRate*sinf(lat_rad);
}

// initialise the earth magnetic field states using declination, suppled roll/pitch
// and magnetometer measurements and return initial attitude quaternion
Quaternion NavEKF2_core::calcQuatAndFieldStates(float roll, float pitch)
{
    Quaternion initQuat;

 
    return initQuat;
}

// zero the attitude covariances, but preserve the variances
void NavEKF2_core::zeroAttCovOnly()
{

}

#endif // HAL_CPU_CLASS
