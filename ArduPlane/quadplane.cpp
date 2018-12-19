#include "Plane.h"

const AP_Param::GroupInfo QuadPlane::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable QuadPlane
    // @Description: This enables QuadPlane functionality, assuming multicopter motors start on output 5. If this is set to 2 then when starting AUTO mode it will initially be in VTOL AUTO mode.
    // @Values: 0:Disable,1:Enable,2:Enable VTOL AUTO
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, QuadPlane, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Group: M_
    // @Path: ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    AP_SUBGROUPVARPTR(motors, "M_", 2, QuadPlane, plane.quadplane.motors_var_info),

    // 3 ~ 8 were used by quadplane attitude control PIDs

    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all VTOL flight modes
    // @Units: cdeg
    // @Range: 1000 8000
    // @User: Advanced
    AP_GROUPINFO("ANGLE_MAX", 10, QuadPlane, aparm.angle_max, 3000),

    // @Param: TRANSITION_MS
    // @DisplayName: Transition time
    // @Description: Transition time in milliseconds after minimum airspeed is reached
    // @Units: ms
    // @Range: 0 30000
    // @User: Advanced
    AP_GROUPINFO("TRANSITION_MS", 11, QuadPlane, transition_time_ms, 5000),

    // 12 ~ 16 were used by position, velocity and acceleration PIDs

    // @Group: P
    // @Path: ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    AP_SUBGROUPPTR(pos_control, "P", 17, QuadPlane, AC_PosControl),

    // @Param: VELZ_MAX
    // @DisplayName: Pilot maximum vertical speed
    // @Description: The maximum vertical velocity the pilot may request in cm/s
    // @Units: cm/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("VELZ_MAX", 18, QuadPlane, pilot_velocity_z_max, 250),
    
    // @Param: ACCEL_Z
    // @DisplayName: Pilot vertical acceleration
    // @Description: The vertical acceleration used when pilot is controlling the altitude
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL_Z",  19, QuadPlane, pilot_accel_z,  250),

    // @Group: WP_
    // @Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    AP_SUBGROUPPTR(wp_nav, "WP_",  20, QuadPlane, AC_WPNav),

    // @Param: RC_SPEED
    // @DisplayName: RC output speed in Hz
    // @Description: This is the PWM refresh rate in Hz for QuadPlane quad motors
    // @Units: Hz
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RC_SPEED", 21, QuadPlane, rc_speed, 490),

    // @Param: THR_MIN_PWM
    // @DisplayName: Minimum PWM output
    // @Description: This is the minimum PWM output for the quad motors
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MIN_PWM", 22, QuadPlane, thr_min_pwm, 1000),

    // @Param: THR_MAX_PWM
    // @DisplayName: Maximum PWM output
    // @Description: This is the maximum PWM output for the quad motors
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MAX_PWM", 23, QuadPlane, thr_max_pwm, 2000),

    // @Param: ASSIST_SPEED
    // @DisplayName: Quadplane assistance speed
    // @Description: This is the speed below which the quad motors will provide stability and lift assistance in fixed wing modes. Zero means no assistance except during transition
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ASSIST_SPEED", 24, QuadPlane, assist_speed, 0),

    // @Param: YAW_RATE_MAX
    // @DisplayName: Maximum yaw rate
    // @Description: This is the maximum yaw rate for pilot input on rudder stick in degrees/second
    // @Units: deg/s
    // @Range: 50 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_RATE_MAX", 25, QuadPlane, yaw_rate_max, 100),

    // @Param: LAND_SPEED
    // @DisplayName: Land speed
    // @Description: The descent speed for the final stage of landing in cm/s
    // @Units: cm/s
    // @Range: 30 200
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("LAND_SPEED", 26, QuadPlane, land_speed_cms, 50),

    // @Param: LAND_FINAL_ALT
    // @DisplayName: Land final altitude
    // @Description: The altitude at which we should switch to Q_LAND_SPEED descent rate
    // @Units: m
    // @Range: 0.5 50
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("LAND_FINAL_ALT", 27, QuadPlane, land_final_alt, 6),

    // 28 was used by THR_MID

    // @Param: TRAN_PIT_MAX
    // @DisplayName: Transition max pitch
    // @Description: Maximum pitch during transition to auto fixed wing flight
    // @User: Standard
    // @Range: 0 30
    // @Units: deg
    // @Increment: 1
    AP_GROUPINFO("TRAN_PIT_MAX", 29, QuadPlane, transition_pitch_max, 3),

    // frame class was moved from 30 when consolidating AP_Motors classes
#define FRAME_CLASS_OLD_IDX 30
    // @Param: FRAME_CLASS
    // @DisplayName: Frame Class
    // @Description: Controls major frame class for multicopter component
    // @Values: 0:Undefined, 1:Quad, 2:Hexa, 3:Octa, 4:OctaQuad, 5:Y6, 7:Tri, 10: TailSitter
    // @User: Standard
    AP_GROUPINFO("FRAME_CLASS", 46, QuadPlane, frame_class, 1),

    // @Param: FRAME_TYPE
    // @DisplayName: Frame Type (+, X or V)
    // @Description: Controls motor mixing for multicopter component
    // @Values: 0:Plus, 1:X, 2:V, 3:H, 4:V-Tail, 5:A-Tail, 10:Y6B, 11:Y6F
    // @User: Standard
    AP_GROUPINFO("FRAME_TYPE", 31, QuadPlane, frame_type, 1),

    // @Param: VFWD_GAIN
    // @DisplayName: Forward velocity hold gain
    // @Description: Controls use of forward motor in vtol modes. If this is zero then the forward motor will not be used for position control in VTOL modes. A value of 0.05 is a good place to start if you want to use the forward motor for position control. No forward motor will be used in QSTABILIZE or QHOVER modes. Use QLOITER for position hold with the forward motor.
    // @Range: 0 0.5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("VFWD_GAIN", 32, QuadPlane, vel_forward.gain, 0),

    // @Param: WVANE_GAIN
    // @DisplayName: Weathervaning gain
    // @Description: This controls the tendency to yaw to face into the wind. A value of 0.1 is to start with and will give a slow turn into the wind. Use a value of 0.4 for more rapid response. The weathervaning works by turning into the direction of roll.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("WVANE_GAIN", 33, QuadPlane, weathervane.gain, 0),

    // @Param: WVANE_MINROLL
    // @DisplayName: Weathervaning min roll
    // @Description: This set the minimum roll in degrees before active weathervaning will start. This may need to be larger if your aircraft has bad roll trim.
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("WVANE_MINROLL", 34, QuadPlane, weathervane.min_roll, 1),

    // @Param: RTL_ALT
    // @DisplayName: QRTL return altitude
    // @Description: The altitude which QRTL mode heads to initially
    // @Units: m
    // @Range: 1 200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RTL_ALT", 35, QuadPlane, qrtl_alt, 15),

    // @Param: RTL_MODE
    // @DisplayName: VTOL RTL mode
    // @Description: If this is set to 1 then an RTL will change to QRTL when within RTL_RADIUS meters of the RTL destination
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("RTL_MODE", 36, QuadPlane, rtl_mode, 0),

    // @Param: TILT_MASK
    // @DisplayName: Tiltrotor mask
    // @Description: This is a bitmask of motors that are tiltable in a tiltrotor (or tiltwing). The mask is in terms of the standard motor order for the frame type.
    // @User: Standard
    AP_GROUPINFO("TILT_MASK", 37, QuadPlane, tilt.tilt_mask, 0),

    // @Param: TILT_RATE_UP
    // @DisplayName: Tiltrotor upwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from forward flight to hover
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("TILT_RATE_UP", 38, QuadPlane, tilt.max_rate_up_dps, 40),

    // @Param: TILT_MAX
    // @DisplayName: Tiltrotor maximum VTOL angle
    // @Description: This is the maximum angle of the tiltable motors at which multicopter control will be enabled. Beyond this angle the plane will fly solely as a fixed wing aircraft and the motors will tilt to their maximum angle at the TILT_RATE
    // @Units: deg
    // @Increment: 1
    // @Range: 20 80
    // @User: Standard
    AP_GROUPINFO("TILT_MAX", 39, QuadPlane, tilt.max_angle_deg, 45),

    // @Param: GUIDED_MODE
    // @DisplayName: Enable VTOL in GUIDED mode
    // @Description: This enables use of VTOL in guided mode. When enabled the aircraft will switch to VTOL flight when the guided destination is reached and hover at the destination.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("GUIDED_MODE", 40, QuadPlane, guided_mode, 0),

    // 41 was used by THR_MIN

    // @Param: ESC_CAL
    // @DisplayName: ESC Calibration
    // @Description: This is used to calibrate the throttle range of the VTOL motors. Please read http://ardupilot.org/plane/docs/quadplane-esc-calibration.html before using. This parameter is automatically set back to 0 on every boot. This parameter only takes effect in QSTABILIZE mode. When set to 1 the output of all motors will come directly from the throttle stick when armed, and will be zero when disarmed. When set to 2 the output of all motors will be maximum when armed and zero when disarmed. Make sure you remove all properllers before using.
    // @Values: 0:Disabled,1:ThrottleInput,2:FullInput
    // @User: Standard
    AP_GROUPINFO("ESC_CAL", 42, QuadPlane, esc_calibration,  0),

    // @Param: VFWD_ALT
    // @DisplayName: Forward velocity alt cutoff
    // @Description: Controls altitude to disable forward velocity assist when below this relative altitude. This is useful to keep the forward velocity propeller from hitting the ground. Rangefinder height data is incorporated when available.
    // @Range: 0 10
    // @Increment: 0.25
    // @User: Standard
    AP_GROUPINFO("VFWD_ALT", 43, QuadPlane, vel_forward_alt_cutoff,  0),

    // @Param: LAND_ICE_CUT
    // @DisplayName: Cut IC engine on landing
    // @Description: This controls stopping an internal combustion engine in the final landing stage of a VTOL. This is important for aircraft where the forward thrust engine may experience prop-strike if left running during landing. This requires the engine controls are enabled using the ICE_* parameters.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("LAND_ICE_CUT", 44, QuadPlane, land_icengine_cut,  1),
    
    // @Param: ASSIST_ANGLE
    // @DisplayName: Quadplane assistance angle
    // @Description: This is the angular error in attitude beyond which the quadplane VTOL motors will provide stability assistance. This will only be used if Q_ASSIST_SPEED is also non-zero. Assistance will be given if the attitude is outside the normal attitude limits by at least 5 degrees and the angular error in roll or pitch is greater than this angle for at least 1 second. Set to zero to disable angle assistance.
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ASSIST_ANGLE", 45, QuadPlane, assist_angle, 30),

    // @Param: TILT_TYPE
    // @DisplayName: Tiltrotor type
    // @Description: This is the type of tiltrotor when TILT_MASK is non-zero. A continuous tiltrotor can tilt the rotors to any angle on demand. A binary tiltrotor assumes a retract style servo where the servo is either fully forward or fully up. In both cases the servo can't move faster than Q_TILT_RATE. A vectored yaw tiltrotor will use the tilt of the motors to control yaw in hover
    // @Values: 0:Continuous,1:Binary,2:VectoredYaw
    AP_GROUPINFO("TILT_TYPE", 47, QuadPlane, tilt.tilt_type, TILT_TYPE_CONTINUOUS),

    // @Param: TAILSIT_ANGLE
    // @DisplayName: Tailsitter transition angle
    // @Description: This is the angle at which tailsitter aircraft will change from VTOL control to fixed wing control.
    // @Range: 5 80
    AP_GROUPINFO("TAILSIT_ANGLE", 48, QuadPlane, tailsitter.transition_angle, 45),

    // @Param: TILT_RATE_DN
    // @DisplayName: Tiltrotor downwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from hover to forward flight. When this is zero the Q_TILT_RATE_UP value is used.
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("TILT_RATE_DN", 49, QuadPlane, tilt.max_rate_down_dps, 0),
        
    // @Param: TAILSIT_INPUT
    // @DisplayName: Tailsitter input type
    // @Description: This controls whether stick input when hovering as a tailsitter follows the conventions for fixed wing hovering or multicopter hovering. When multicopter input is selected the roll stick will roll the aircraft in earth frame and yaw stick will yaw in earth frame. When using fixed wing input the roll and yaw sticks will control the aircraft in body frame.
    // @Values: 0:MultiCopterInput,1:FixedWingInput
    AP_GROUPINFO("TAILSIT_INPUT", 50, QuadPlane, tailsitter.input_type, TAILSITTER_INPUT_MULTICOPTER),

    // @Param: TAILSIT_MASK
    // @DisplayName: Tailsitter input mask
    // @Description: This controls what channels have full manual control when hovering as a tailsitter and the Q_TAILSIT_MASKCH channel in high. This can be used to teach yourself to prop-hang a 3D plane by learning one or more channels at a time.
    // @Bitmask: 0:Aileron,1:Elevator,2:Throttle,3:Rudder
    AP_GROUPINFO("TAILSIT_MASK", 51, QuadPlane, tailsitter.input_mask, 0),

    // @Param: TAILSIT_MASKCH
    // @DisplayName: Tailsitter input mask channel
    // @Description: This controls what input channel will activate the Q_TAILSIT_MASK mask. When this channel goes above 1700 then the pilot will have direct manual control of the output channels specified in Q_TAILSIT_MASK. Set to zero to disable.
    // @Values: 0:Disabled,1:Channel1,2:Channel2,3:Channel3,4:Channel4,5:Channel5,6:Channel6,7:Channel7,8:Channel8
    AP_GROUPINFO("TAILSIT_MASKCH", 52, QuadPlane, tailsitter.input_mask_chan, 0),

    // @Param: TAILSIT_VFGAIN
    // @DisplayName: Tailsitter vector thrust gain in forward flight
    // @Description: This controls the amount of vectored thrust control used in forward flight for a vectored tailsitter
    // @Range: 0 1
    // @Increment: 0.01
    AP_GROUPINFO("TAILSIT_VFGAIN", 53, QuadPlane, tailsitter.vectored_forward_gain, 0),

    // @Param: TAILSIT_VHGAIN
    // @DisplayName: Tailsitter vector thrust gain in hover
    // @Description: This controls the amount of vectored thrust control used in hover for a vectored tailsitter
    // @Range: 0 1
    // @Increment: 0.01
    AP_GROUPINFO("TAILSIT_VHGAIN", 54, QuadPlane, tailsitter.vectored_hover_gain, 0.5),

    // @Param: TILT_YAW_ANGLE
    // @DisplayName: Tilt minimum angle for vectored yaw
    // @Description: This is the angle of the tilt servos when in VTOL mode and at minimum output. This needs to be set for Q_TILT_TYPE=3 to enable vectored control for yaw of tricopter tilt quadplanes.
    // @Range: 0 30
    AP_GROUPINFO("TILT_YAW_ANGLE", 55, QuadPlane, tilt.tilt_yaw_angle, 0),

    // @Param: TAILSIT_VHPOW
    // @DisplayName: Tailsitter vector thrust gain power
    // @Description: This controls the amount of extra pitch given to the vectored control when at high pitch errors
    // @Range: 0 4
    // @Increment: 0.1
    AP_GROUPINFO("TAILSIT_VHPOW", 56, QuadPlane, tailsitter.vectored_hover_power, 2.5),

    // @Param: MAV_TYPE
    // @DisplayName: MAVLink type identifier
    // @Description: This controls the mavlink type given in HEARTBEAT messages. For some GCS types a particular setting will be needed for correct operation.
    // @Values: 0:AUTO,1:FIXED_WING,2:QUADROTOR,3:COAXIAL,4:HELICOPTER,7:AIRSHIP,8:FREE_BALLOON,9:ROCKET,10:GROUND_ROVER,11:SURFACE_BOAT,12:SUBMARINE,16:FLAPPING_WING,17:KITE,19:VTOL_DUOROTOR,20:VTOL_QUADROTOR,21:VTOL_TILTROTOR
    AP_GROUPINFO("MAV_TYPE", 57, QuadPlane, mav_type, 0),

    // @Param: OPTIONS
    // @DisplayName: quadplane options
    // @Description: This provides a set of additional control options for quadplanes. LevelTransition means that the wings should be held level to within LEVEL_ROLL_LIMIT degrees during transition to fixed wing flight, and the vehicle will not use the vertical lift motors to climb during the transition. If AllowFWTakeoff bit is not set then fixed wing takeoff on quadplanes will instead perform a VTOL takeoff. If AllowFWLand bit is not set then fixed wing land on quadplanes will instead perform a VTOL land. If respect takeoff frame is not set the vehicle will interpret all takeoff waypoints as an altitude above the corrent position.
    // @Bitmask: 0:LevelTransition,1:AllowFWTakeoff,2:AllowFWLand,3:Respect takeoff frame types,4:Use a fixed wing approach for VTOL landings
    AP_GROUPINFO("OPTIONS", 58, QuadPlane, options, 0),

    AP_SUBGROUPEXTENSION("",59, QuadPlane, var_info2),

    AP_GROUPEND
};

// second table of user settable parameters for quadplanes, this
// allows us to go beyond the 64 parameter limit
const AP_Param::GroupInfo QuadPlane::var_info2[] = {
    // @Param: TRANS_DECEL
    // @DisplayName: Transition deceleration
    // @Description: This is deceleration rate that will be used in calculating the stopping distance when transitioning from fixed wing flight to multicopter flight.
    // @Units: m/s/s
    // @Increment: 0.1
    // @Range: 0.2 5
    // @User: Standard
    AP_GROUPINFO("TRANS_DECEL", 1, QuadPlane, transition_decel, 2.0),

    // @Group: LOIT_
    // @Path: ../libraries/AC_WPNav/AC_Loiter.cpp
    AP_SUBGROUPPTR(loiter_nav, "LOIT_",  2, QuadPlane, AC_Loiter),

    // @Param: TAILSIT_THSCMX
    // @DisplayName: Maximum control throttle scaling value
    // @Description: Maximum value of throttle scaling for tailsitter velocity scaling, reduce this value to remove low thorottle D ossilaitons 
    // @Range: 1 5
    // @User: Standard
    AP_GROUPINFO("TAILSIT_THSCMX", 3, QuadPlane, tailsitter.throttle_scale_max, 5),

    // @Param: TRIM_PITCH
    // @DisplayName: Quadplane AHRS trim pitch
    // @Description: This sets the compensation for the pitch angle trim difference between forward and vertical flight pitch, NOTE! this is relative to forward flight trim not mounting locaiton. For tailsitters this is relative to a baseline of 90 degrees.
    // @Units: deg
    // @Range: -10 +10
    // @Increment: 0.1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TRIM_PITCH", 4, QuadPlane, ahrs_trim_pitch, 0),

    // @Param: TAILSIT_RLL_MX
    // @DisplayName: Maximum Roll angle
    // @Description: Maximum Allowed roll angle for tailsitters. If this is zero then Q_ANGLE_MAX is used.
    // @Units: deg
    // @Range: 0 80
    // @User: Standard
    AP_GROUPINFO("TAILSIT_RLL_MX", 5, QuadPlane, tailsitter.max_roll_angle, 0),

#if QAUTOTUNE_ENABLED
    // @Group: AUTOTUNE_
    // @Path: qautotune.cpp
    AP_SUBGROUPINFO(qautotune, "AUTOTUNE_",  6, QuadPlane, QAutoTune),
#endif

    // @Param: FW_LND_APR_RAD
    // @DisplayName: Quadplane fixed wing landing approach radius
    // @Description: This provides the radius used, when using a fixed wing landing approach. If set to 0 then the WP_LOITER_RAD will be selected.
    // @Units: m
    // @Range: 0 200
    // @Increment: 5
    // @User: Advanced
    AP_GROUPINFO("FW_LND_APR_RAD", 7, QuadPlane, fw_land_approach_radius, 0),

    AP_GROUPEND
};

/*
  defaults for all quadplanes
 */
static const struct AP_Param::defaults_table_struct defaults_table[] = {
    { "Q_A_RAT_RLL_P",    0.25 },
    { "Q_A_RAT_RLL_I",    0.25 },
    { "Q_A_RAT_RLL_FILT", 10.0 },
    { "Q_A_RAT_PIT_P",    0.25 },
    { "Q_A_RAT_PIT_I",    0.25 },
    { "Q_A_RAT_PIT_FILT", 10.0 },
    { "Q_M_SPOOL_TIME",   0.25 },
    { "Q_LOIT_ANG_MAX",   15.0 },
    { "Q_LOIT_ACC_MAX",   250.0 },
    { "Q_LOIT_BRK_ACCEL", 50.0 },
    { "Q_LOIT_BRK_JERK",  250 },
    { "Q_LOIT_SPEED",     500 },
};

/*
  extra defaults for tailsitters
 */
static const struct AP_Param::defaults_table_struct defaults_table_tailsitter[] = {
    { "KFF_RDDRMIX",       0.02 },
    { "Q_A_RAT_PIT_FF",    0.2 },
    { "Q_A_RAT_YAW_FF",    0.2 },
    { "Q_A_RAT_YAW_I",    0.18 },
    { "LIM_PITCH_MAX",    3000 },
    { "LIM_PITCH_MIN",    -3000 },
    { "MIXING_GAIN",      1.0 },
    { "RUDD_DT_GAIN",      10 },
    { "Q_TRANSITION_MS",   2000 },
};

/*
  conversion table for quadplane parameters
 */
const AP_Param::ConversionInfo q_conversion_table[] = {
    { Parameters::k_param_quadplane, 4044, AP_PARAM_FLOAT, "Q_P_POSZ_P" },     //  Q_PZ_P
    { Parameters::k_param_quadplane, 4045, AP_PARAM_FLOAT, "Q_P_POSXY_P"},     //  Q_PXY_P
    { Parameters::k_param_quadplane, 4046, AP_PARAM_FLOAT, "Q_P_VELXY_P"},     //  Q_VXY_P
    { Parameters::k_param_quadplane, 78,   AP_PARAM_FLOAT, "Q_P_VELXY_I"},     //  Q_VXY_I
    { Parameters::k_param_quadplane, 142,  AP_PARAM_FLOAT, "Q_P_VELXY_IMAX"},  //  Q_VXY_IMAX
    { Parameters::k_param_quadplane, 206,  AP_PARAM_FLOAT, "Q_P_VELXY_FILT"},  //  Q_VXY_FILT_HZ
    { Parameters::k_param_quadplane, 4047, AP_PARAM_FLOAT, "Q_P_VELZ_P"},      //  Q_VZ_P
    { Parameters::k_param_quadplane, 4048, AP_PARAM_FLOAT, "Q_P_ACCZ_P"},      //  Q_AZ_P
    { Parameters::k_param_quadplane, 80,   AP_PARAM_FLOAT, "Q_P_ACCZ_I"},      //  Q_AZ_I
    { Parameters::k_param_quadplane, 144,  AP_PARAM_FLOAT, "Q_P_ACCZ_D"},      //  Q_AZ_D
    { Parameters::k_param_quadplane, 336,  AP_PARAM_FLOAT, "Q_P_ACCZ_IMAX"},   //  Q_AZ_IMAX
    { Parameters::k_param_quadplane, 400,  AP_PARAM_FLOAT, "Q_P_ACCZ_FILT"},   //  Q_AZ_FILT
    { Parameters::k_param_quadplane, 464,  AP_PARAM_FLOAT, "Q_P_ACCZ_FF"},     //  Q_AZ_FF
    { Parameters::k_param_quadplane, 276,  AP_PARAM_FLOAT, "Q_LOIT_SPEED"},    //  Q_WP_LOIT_SPEED
    { Parameters::k_param_quadplane, 468,  AP_PARAM_FLOAT, "Q_LOIT_BRK_JERK" },//  Q_WP_LOIT_JERK
    { Parameters::k_param_quadplane, 532,  AP_PARAM_FLOAT, "Q_LOIT_ACC_MAX" }, //  Q_WP_LOIT_MAXA
    { Parameters::k_param_quadplane, 596,  AP_PARAM_FLOAT, "Q_LOIT_BRK_ACCEL" },// Q_WP_LOIT_MINA
};


QuadPlane::QuadPlane(AP_AHRS_NavEKF &_ahrs) :
    ahrs(_ahrs)
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::setup_object_defaults(this, var_info2);
}


// setup default motors for the frame class
void QuadPlane::setup_default_channels(uint8_t num_motors)
{

}
    

bool QuadPlane::setup(void)
{
    return false;
}

/*
  setup default parameters from defaults_table
 */
void QuadPlane::setup_defaults(void)
{

}

// run ESC calibration
void QuadPlane::run_esc_calibration(void)
{

}


// init quadplane stabilize mode 
void QuadPlane::init_stabilize(void)
{
    throttle_wait = false;
}


/*
  ask the multicopter attitude control to match the roll and pitch rates being demanded by the
  fixed wing controller if not in a pure VTOL mode
 */
void QuadPlane::multicopter_attitude_rate_update(float yaw_rate_cds)
{

}

// hold in stabilize with given throttle
void QuadPlane::hold_stabilize(float throttle_in)
{    

}

// quadplane stabilize mode
void QuadPlane::control_stabilize(void)
{

}

// run the multicopter Z controller
void QuadPlane::run_z_controller(void)
{
   
}

/*
  check if we should relax the attitude controllers

  We relax them whenever we will be using them after a period of
  inactivity
 */
void QuadPlane::check_attitude_relax(void)
{

}

// init quadplane hover mode 
void QuadPlane::init_hover(void)
{

}

/*
  check for an EKF yaw reset
 */
void QuadPlane::check_yaw_reset(void)
{

}

/*
  hold hover with target climb rate
 */
void QuadPlane::hold_hover(float target_climb_rate)
{

}

/*
  control QHOVER mode
 */
void QuadPlane::control_hover(void)
{

}

void QuadPlane::init_loiter(void)
{

}

void QuadPlane::init_land(void)
{

}


// helper for is_flying()
bool QuadPlane::is_flying(void)
{
    return false;
}

// crude landing detector to prevent tipover
bool QuadPlane::should_relax(void)
{
   return true;
}

// see if we are flying in vtol
bool QuadPlane::is_flying_vtol(void) const
{

    return false;
}

/*
  smooth out descent rate for landing to prevent a jerk as we get to
  land_final_alt. 
 */
float QuadPlane::landing_descent_rate_cms(float height_above_ground) const
{
    return 0.0f;
}


// run quadplane loiter controller
void QuadPlane::control_loiter()
{
 
}

/*
  get pilot input yaw rate in cd/s
 */
float QuadPlane::get_pilot_input_yaw_rate_cds(void) const
{
    return 0;
}

/*
  get overall desired yaw rate in cd/s
 */
float QuadPlane::get_desired_yaw_rate_cds(void)
{
    return 0;
}

// get pilot desired climb rate in cm/s
float QuadPlane::get_pilot_desired_climb_rate_cms(void) const
{
    return 0;
}


/*
  initialise throttle_wait based on throttle and is_flying()
 */
void QuadPlane::init_throttle_wait(void)
{
    
}
    
// set motor arming
void QuadPlane::set_armed(bool armed)
{

}


/*
  estimate desired climb rate for assistance (in cm/s)
 */
float QuadPlane::assist_climb_rate_cms(void) const
{
    return 0.0f;
}

/*
  calculate desired yaw rate for assistance
 */
float QuadPlane::desired_auto_yaw_rate_cds(void) const
{
    return 0.0f;
}

/*
  return true if the quadplane should provide stability assistance
 */
bool QuadPlane::assistance_needed(float aspeed)
{
    return false;
}

/*
  update for transition from quadplane to fixed wing mode
 */
void QuadPlane::update_transition(void)
{
 
}

/*
  update motor output for quadplane
 */
void QuadPlane::update(void)
{
 
}

/*
  see if motors should be shutdown. If they should be then change AP_Motors state to 
  AP_Motors::DESIRED_SHUT_DOWN

  This is a safety check to prevent accidental motor runs on the
  ground, such as if RC fails and QRTL is started
 */
void QuadPlane::check_throttle_suppression(void)
{
 
}

/*
  output motors and do any copter needed
 */
void QuadPlane::motors_output(bool run_rate_controller)
{

}

/*
  update control mode for quadplane modes
 */
void QuadPlane::control_run(void)
{

}

/*
  enter a quadplane mode
 */
bool QuadPlane::init_mode(void)
{
    return false;
}

/*
  handle a MAVLink DO_VTOL_TRANSITION
 */
bool QuadPlane::handle_do_vtol_transition(enum MAV_VTOL_STATE state)
{
    return false;
}

/*
  are we in a VTOL auto state?
 */
bool QuadPlane::in_vtol_auto(void) const
{
    return false;
}

/*
  are we in a VTOL mode?
 */
bool QuadPlane::in_vtol_mode(void) const
{
    return false;
}


/*
  main landing controller. Used for landing and RTL.
 */
void QuadPlane::vtol_position_controller(void)
{

}


/*
  setup the target position based on plane.next_WP_loc
 */
void QuadPlane::setup_target_position(void)
{

}

/*
  run takeoff controller to climb vertically
 */
void QuadPlane::takeoff_controller(void)
{

}

/*
  run waypoint controller between prev_WP_loc and next_WP_loc
 */
void QuadPlane::waypoint_controller(void)
{

}


/*
  handle auto-mode when auto_state.vtol_mode is true
 */
void QuadPlane::control_auto(const Location &loc)
{
 
}

/*
  handle QRTL mode
 */
void QuadPlane::control_qrtl(void)
{

}

/*
  handle QRTL mode
 */
void QuadPlane::init_qrtl(void)
{

}

/*
  start a VTOL takeoff
 */
bool QuadPlane::do_vtol_takeoff(const AP_Mission::Mission_Command& cmd)
{
    return false;
}


/*
  start a VTOL landing
 */
bool QuadPlane::do_vtol_land(const AP_Mission::Mission_Command& cmd)
{
    return false;
}

/*
  check if a VTOL takeoff has completed
 */
bool QuadPlane::verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd)
{
    return false;
}

/*
  check if a landing is complete
 */
void QuadPlane::check_land_complete(void)
{

}

/*
  check if a VTOL landing has completed
 */
bool QuadPlane::verify_vtol_land(void)
{
    return true;
}

// Write a control tuning packet
void QuadPlane::Log_Write_QControl_Tuning()
{

}


/*
  calculate the forward throttle percentage. The forward throttle can
  be used to assist with position hold and with landing approach. It
  reduces the need for down pitch which reduces load on the vertical
  lift motors.
 */
int8_t QuadPlane::forward_throttle_pct(void)
{
   return 0;
}

/*
  get weathervaning yaw rate in cd/s
 */
float QuadPlane::get_weathervane_yaw_rate_cds(void)
{
    return 0;
}

/*
  start guided mode control
 */
void QuadPlane::guided_start(void)
{

}

/*
  update guided mode control
 */
void QuadPlane::guided_update(void)
{

}

void QuadPlane::afs_terminate(void)
{

}

/*
  return true if we should do guided mode loitering using VTOL motors
 */
bool QuadPlane::guided_mode_enabled(void)
{
    return false;
}

/*
  set altitude target to current altitude
 */
void QuadPlane::set_alt_target_current(void)
{

}

/*
  adjust the altitude target to the given target, moving it slowly
 */
void QuadPlane::adjust_alt_target(float altitude_cm)
{

}

// user initiated takeoff for guided mode
bool QuadPlane::do_user_takeoff(float takeoff_altitude)
{
   return false;
}

// return true if the wp_nav controller is being updated
bool QuadPlane::using_wp_nav(void) const
{
    return false;
}

/*
  return mav_type for heartbeat
 */
MAV_TYPE QuadPlane::get_mav_type(void) const
{
    if (mav_type.get() == 0) {
        return MAV_TYPE_FIXED_WING;
    }
    return MAV_TYPE(mav_type.get());
}

/*
  return true if current mission item is a vtol takeoff
*/
bool QuadPlane::is_vtol_takeoff(uint16_t id) const
{
    if (id == MAV_CMD_NAV_VTOL_TAKEOFF) {
        return true;
    }
    if (id == MAV_CMD_NAV_TAKEOFF && available() && (options & OPTION_ALLOW_FW_TAKEOFF) == 0) {
        // treat fixed wing takeoff as VTOL takeoff
        return true;
    }
    return false;
}

/*
  return true if current mission item is a vtol land
*/
bool QuadPlane::is_vtol_land(uint16_t id) const
{
    if (id == MAV_CMD_NAV_VTOL_LAND) {
        if (options & QuadPlane::OPTION_MISSION_LAND_FW_APPROACH) {
            return plane.vtol_approach_s.approach_stage == Plane::Landing_ApproachStage::VTOL_LANDING;
        } else {
            return true;
        }
    }
    if (id == MAV_CMD_NAV_LAND && available() && (options & OPTION_ALLOW_FW_LAND) == 0) {
        // treat fixed wing land as VTOL land
        return true;
    }
    return false;
}

/*
  return true if we are in a transition to fwd flight from hover
 */
bool QuadPlane::in_transition(void) const
{
    return available() && assisted_flight &&
        (transition_state == TRANSITION_AIRSPEED_WAIT ||
         transition_state == TRANSITION_TIMER);
}

/*
  calculate current stopping distance for a quadplane in fixed wing flight
 */
float QuadPlane::stopping_distance(void)
{
    // use v^2/(2*accel). This is only quite approximate as the drag
    // varies with pitch, but it gives something for the user to
    // control the transition distance in a reasonable way
    return plane.ahrs.groundspeed_vector().length_squared() / (2 * transition_decel);
}
