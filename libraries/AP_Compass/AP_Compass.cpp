#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/I2CDevice.h>
#endif
#if HAL_WITH_UAVCAN
#include "AP_Compass_UAVCAN.h"
#endif
#include "AP_Compass_MMC3416.h"
#include "AP_Compass_MAG3110.h"
#include "AP_Compass.h"
#include "Compass_learn.h"

extern AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
#define COMPASS_LEARN_DEFAULT Compass::LEARN_NONE
#else
#define COMPASS_LEARN_DEFAULT Compass::LEARN_INTERNAL
#endif

#ifndef AP_COMPASS_OFFSETS_MAX_DEFAULT
#define AP_COMPASS_OFFSETS_MAX_DEFAULT 1250
#endif

#ifndef HAL_COMPASS_FILTER_DEFAULT
 #define HAL_COMPASS_FILTER_DEFAULT 0 // turned off by default
#endif

#ifndef HAL_COMPASS_AUTO_ROT_DEFAULT
#define HAL_COMPASS_AUTO_ROT_DEFAULT 2
#endif

const AP_Param::GroupInfo Compass::var_info[] = {
    // index 0 was used for the old orientation matrix

    // @Param: OFS_X
    // @DisplayName: Compass offsets in milligauss on the X axis
    // @Description: Offset to be added to the compass x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced

    // @Param: OFS_Y
    // @DisplayName: Compass offsets in milligauss on the Y axis
    // @Description: Offset to be added to the compass y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced

    // @Param: OFS_Z
    // @DisplayName: Compass offsets in milligauss on the Z axis
    // @Description: Offset to be added to the compass z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OFS",    1, Compass, _state[0].offset, 0),

    // @Param: DEC
    // @DisplayName: Compass declination
    // @Description: An angle to compensate between the true north and magnetic north
    // @Range: -3.142 3.142
    // @Units: rad
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DEC",    2, Compass, _declination, 0),

    // @Param: LEARN
    // @DisplayName: Learn compass offsets automatically
    // @Description: Enable or disable the automatic learning of compass offsets. You can enable learning either using a compass-only method that is suitable only for fixed wing aircraft or using the offsets learnt by the active EKF state estimator. If this option is enabled then the learnt offsets are saved when you disarm the vehicle.
    // @Values: 0:Disabled,1:Internal-Learning,2:EKF-Learning
    // @User: Advanced
    AP_GROUPINFO("LEARN",  3, Compass, _learn, COMPASS_LEARN_DEFAULT),

    // @Param: USE
    // @DisplayName: Use compass for yaw
    // @Description: Enable or disable the use of the compass (instead of the GPS) for determining heading
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE",    4, Compass, _state[0].use_for_yaw, 1), // true if used for DCM yaw

    // @Param: AUTODEC
    // @DisplayName: Auto Declination
    // @Description: Enable or disable the automatic calculation of the declination based on gps location
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("AUTODEC",5, Compass, _auto_declination, 1),

    // @Param: MOTCT
    // @DisplayName: Motor interference compensation type
    // @Description: Set motor interference compensation type to disabled, throttle or current.  Do not change manually.
    // @Values: 0:Disabled,1:Use Throttle,2:Use Current
    // @User: Advanced
    AP_GROUPINFO("MOTCT",    6, Compass, _motor_comp_type, AP_COMPASS_MOT_COMP_DISABLED),

    // @Param: MOT_X
    // @DisplayName: Motor interference compensation for body frame X axis
    // @Description: Multiplied by the current throttle and added to the compass's x-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced

    // @Param: MOT_Y
    // @DisplayName: Motor interference compensation for body frame Y axis
    // @Description: Multiplied by the current throttle and added to the compass's y-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced

    // @Param: MOT_Z
    // @DisplayName: Motor interference compensation for body frame Z axis
    // @Description: Multiplied by the current throttle and added to the compass's z-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MOT",    7, Compass, _state[0].motor_compensation, 0),

    // @Param: ORIENT
    // @DisplayName: Compass orientation
    // @Description: The orientation of the first external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw135,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll180,39:Pitch315,40:Roll90Pitch315
    // @User: Advanced
    AP_GROUPINFO("ORIENT", 8, Compass, _state[0].orientation, ROTATION_NONE),

    // @Param: EXTERNAL
    // @DisplayName: Compass is attached via an external cable
    // @Description: Configure compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. Set to 1 if the compass is externally connected. When externally connected the COMPASS_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
    // @Values: 0:Internal,1:External,2:ForcedExternal
    // @User: Advanced
    AP_GROUPINFO("EXTERNAL", 9, Compass, _state[0].external, 0),

    // @Param: OFS2_X
    // @DisplayName: Compass2 offsets in milligauss on the X axis
    // @Description: Offset to be added to compass2's x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced

    // @Param: OFS2_Y
    // @DisplayName: Compass2 offsets in milligauss on the Y axis
    // @Description: Offset to be added to compass2's y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced

    // @Param: OFS2_Z
    // @DisplayName: Compass2 offsets in milligauss on the Z axis
    // @Description: Offset to be added to compass2's z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OFS2",    10, Compass, _state[1].offset, 0),

    // @Param: MOT2_X
    // @DisplayName: Motor interference compensation to compass2 for body frame X axis
    // @Description: Multiplied by the current throttle and added to compass2's x-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced

    // @Param: MOT2_Y
    // @DisplayName: Motor interference compensation to compass2 for body frame Y axis
    // @Description: Multiplied by the current throttle and added to compass2's y-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced

    // @Param: MOT2_Z
    // @DisplayName: Motor interference compensation to compass2 for body frame Z axis
    // @Description: Multiplied by the current throttle and added to compass2's z-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MOT2",    11, Compass, _state[1].motor_compensation, 0),

    // @Param: PRIMARY
    // @DisplayName: Choose primary compass
    // @Description: If more than one compass is available, this selects which compass is the primary. When external compasses are connected, they will be ordered first. NOTE: If no external compass is attached, this parameter is ignored.
    // @Values: 0:FirstCompass,1:SecondCompass,2:ThirdCompass
    // @User: Advanced
    AP_GROUPINFO("PRIMARY", 12, Compass, _primary, 0),

    // @Param: OFS3_X
    // @DisplayName: Compass3 offsets in milligauss on the X axis
    // @Description: Offset to be added to compass3's x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced

    // @Param: OFS3_Y
    // @DisplayName: Compass3 offsets in milligauss on the Y axis
    // @Description: Offset to be added to compass3's y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced

    // @Param: OFS3_Z
    // @DisplayName: Compass3 offsets in milligauss on the Z axis
    // @Description: Offset to be added to compass3's z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Units: mGauss
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OFS3",    13, Compass, _state[2].offset, 0),

    // @Param: MOT3_X
    // @DisplayName: Motor interference compensation to compass3 for body frame X axis
    // @Description: Multiplied by the current throttle and added to compass3's x-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced

    // @Param: MOT3_Y
    // @DisplayName: Motor interference compensation to compass3 for body frame Y axis
    // @Description: Multiplied by the current throttle and added to compass3's y-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced

    // @Param: MOT3_Z
    // @DisplayName: Motor interference compensation to compass3 for body frame Z axis
    // @Description: Multiplied by the current throttle and added to compass3's z-axis values to compensate for motor interference (Offset per Amp or at Full Throttle)
    // @Range: -1000 1000
    // @Units: mGauss/A
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MOT3",    14, Compass, _state[2].motor_compensation, 0),

    // @Param: DEV_ID
    // @DisplayName: Compass device id
    // @Description: Compass device id.  Automatically detected, do not set manually
    // @User: Advanced
    AP_GROUPINFO("DEV_ID",  15, Compass, _state[0].dev_id, 0),

    // @Param: DEV_ID2
    // @DisplayName: Compass2 device id
    // @Description: Second compass's device id.  Automatically detected, do not set manually
    // @User: Advanced
    AP_GROUPINFO("DEV_ID2", 16, Compass, _state[1].dev_id, 0),

    // @Param: DEV_ID3
    // @DisplayName: Compass3 device id
    // @Description: Third compass's device id.  Automatically detected, do not set manually
    // @User: Advanced
    AP_GROUPINFO("DEV_ID3", 17, Compass, _state[2].dev_id, 0),

    // @Param: USE2
    // @DisplayName: Compass2 used for yaw
    // @Description: Enable or disable the second compass for determining heading.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE2",    18, Compass, _state[1].use_for_yaw, 1),

    // @Param: ORIENT2
    // @DisplayName: Compass2 orientation
    // @Description: The orientation of a second external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw135,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll180,39:Pitch315,40:Roll90Pitch315
    // @User: Advanced
    AP_GROUPINFO("ORIENT2", 19, Compass, _state[1].orientation, ROTATION_NONE),

    // @Param: EXTERN2
    // @DisplayName: Compass2 is attached via an external cable
    // @Description: Configure second compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
    // @Values: 0:Internal,1:External,2:ForcedExternal
    // @User: Advanced
    AP_GROUPINFO("EXTERN2",20, Compass, _state[1].external, 0),

    // @Param: USE3
    // @DisplayName: Compass3 used for yaw
    // @Description: Enable or disable the third compass for determining heading.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE3",    21, Compass, _state[2].use_for_yaw, 1),

    // @Param: ORIENT3
    // @DisplayName: Compass3 orientation
    // @Description: The orientation of a third external compass relative to the vehicle frame. This value will be ignored unless this compass is set as an external compass. When set correctly in the northern hemisphere, pointing the nose and right side down should increase the MagX and MagY values respectively. Rolling the vehicle upside down should decrease the MagZ value. For southern hemisphere, switch increase and decrease. NOTE: For internal compasses, AHRS_ORIENT is used.
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw90,19:Roll90Yaw135,20:Roll270,21:Roll270Yaw45,22:Roll270Yaw90,23:Roll270Yaw135,24:Pitch90,25:Pitch270,26:Pitch180Yaw90,27:Pitch180Yaw270,28:Roll90Pitch90,29:Roll180Pitch90,30:Roll270Pitch90,31:Roll90Pitch180,32:Roll270Pitch180,33:Roll90Pitch270,34:Roll180Pitch270,35:Roll270Pitch270,36:Roll90Pitch180Yaw90,37:Roll90Yaw270,38:Yaw293Pitch68Roll180,39:Pitch315,40:Roll90Pitch315
    // @User: Advanced
    AP_GROUPINFO("ORIENT3", 22, Compass, _state[2].orientation, ROTATION_NONE),

    // @Param: EXTERN3
    // @DisplayName: Compass3 is attached via an external cable
    // @Description: Configure third compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
    // @Values: 0:Internal,1:External,2:ForcedExternal
    // @User: Advanced
    AP_GROUPINFO("EXTERN3",23, Compass, _state[2].external, 0),

    // @Param: DIA_X
    // @DisplayName: Compass soft-iron diagonal X component
    // @Description: DIA_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA_Y
    // @DisplayName: Compass soft-iron diagonal Y component
    // @Description: DIA_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA_Z
    // @DisplayName: Compass soft-iron diagonal Z component
    // @Description: DIA_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("DIA",    24, Compass, _state[0].diagonals, 0),

    // @Param: ODI_X
    // @DisplayName: Compass soft-iron off-diagonal X component
    // @Description: ODI_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI_Y
    // @DisplayName: Compass soft-iron off-diagonal Y component
    // @Description: ODI_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI_Z
    // @DisplayName: Compass soft-iron off-diagonal Z component
    // @Description: ODI_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("ODI",    25, Compass, _state[0].offdiagonals, 0),

    // @Param: DIA2_X
    // @DisplayName: Compass2 soft-iron diagonal X component
    // @Description: DIA_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA2_Y
    // @DisplayName: Compass2 soft-iron diagonal Y component
    // @Description: DIA_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA2_Z
    // @DisplayName: Compass2 soft-iron diagonal Z component
    // @Description: DIA_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("DIA2",    26, Compass, _state[1].diagonals, 0),

    // @Param: ODI2_X
    // @DisplayName: Compass2 soft-iron off-diagonal X component
    // @Description: ODI_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI2_Y
    // @DisplayName: Compass2 soft-iron off-diagonal Y component
    // @Description: ODI_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI2_Z
    // @DisplayName: Compass2 soft-iron off-diagonal Z component
    // @Description: ODI_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("ODI2",    27, Compass, _state[1].offdiagonals, 0),

    // @Param: DIA3_X
    // @DisplayName: Compass3 soft-iron diagonal X component
    // @Description: DIA_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA3_Y
    // @DisplayName: Compass3 soft-iron diagonal Y component
    // @Description: DIA_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: DIA3_Z
    // @DisplayName: Compass3 soft-iron diagonal Z component
    // @Description: DIA_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("DIA3",    28, Compass, _state[2].diagonals, 0),

    // @Param: ODI3_X
    // @DisplayName: Compass3 soft-iron off-diagonal X component
    // @Description: ODI_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI3_Y
    // @DisplayName: Compass3 soft-iron off-diagonal Y component
    // @Description: ODI_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced

    // @Param: ODI3_Z
    // @DisplayName: Compass3 soft-iron off-diagonal Z component
    // @Description: ODI_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
    // @User: Advanced
    AP_GROUPINFO("ODI3",    29, Compass, _state[2].offdiagonals, 0),

    // @Param: CAL_FIT
    // @DisplayName: Compass calibration fitness
    // @Description: This controls the fitness level required for a successful compass calibration. A lower value makes for a stricter fit (less likely to pass). This is the value used for the primary magnetometer. Other magnetometers get double the value.
    // @Range: 4 32
    // @Values: 4:Very Strict,8:Strict,16:Default,32:Relaxed
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("CAL_FIT", 30, Compass, _calibration_threshold, AP_COMPASS_CALIBRATION_FITNESS_DEFAULT),

    // @Param: OFFS_MAX
    // @DisplayName: Compass maximum offset
    // @Description: This sets the maximum allowed compass offset in calibration and arming checks
    // @Range: 500 3000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OFFS_MAX", 31, Compass, _offset_max, AP_COMPASS_OFFSETS_MAX_DEFAULT),

    // @Group: PMOT
    // @Path: Compass_PerMotor.cpp
    AP_SUBGROUPINFO(_per_motor, "PMOT", 32, Compass, Compass_PerMotor),

    // @Param: TYPEMASK
    // @DisplayName: Compass disable driver type mask
    // @Description: This is a bitmask of driver types to disable. If a driver type is set in this mask then that driver will not try to find a sensor at startup
    // @Bitmask: 0:HMC5883,1:LSM303D,2:AK8963,3:BMM150,4:LSM9DS1,5:LIS3MDL,6:AK09916,7:IST8310,8:ICM20948,9:MMC3416,11:UAVCAN,12:QMC5883,14:MAG3110,15:IST8308
    // @User: Advanced
    AP_GROUPINFO("TYPEMASK", 33, Compass, _driver_type_mask, 0),

    // @Param: FLTR_RNG
    // @DisplayName: Range in which sample is accepted
    // @Description: This sets the range around the average value that new samples must be within to be accepted. This can help reduce the impact of noise on sensors that are on long I2C cables. The value is a percentage from the average value. A value of zero disables this filter.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    AP_GROUPINFO("FLTR_RNG", 34, Compass, _filter_range, HAL_COMPASS_FILTER_DEFAULT),

    // @Param: AUTO_ROT
    // @DisplayName: Automatically check orientation
    // @Description: When enabled this will automatically check the orientation of compasses on successful completion of compass calibration. If set to 2 then external compasses will have their orientation automatically corrected.
    // @Values: 0:Disabled,1:CheckOnly,2:CheckAndFix
    AP_GROUPINFO("AUTO_ROT", 35, Compass, _rotate_auto, HAL_COMPASS_AUTO_ROT_DEFAULT),

    // @Param: EXP_DID
    // @DisplayName: Compass device id expected
    // @Description: The expected value of COMPASS_DEV_ID, used by arming checks. Setting this to -1 means "don't care."
    // @User: Advanced
    AP_GROUPINFO("EXP_DID",  36, Compass, _state[0].expected_dev_id, -1),

    // @Param: EXP_DID2
    // @DisplayName: Compass2 device id expected
    // @Description: The expected value of COMPASS_DEV_ID2, used by arming checks. Setting this to -1 means "don't care."
    // @User: Advanced
    AP_GROUPINFO("EXP_DID2", 37, Compass, _state[1].expected_dev_id, -1),

    // @Param: EXP_DID3
    // @DisplayName: Compass3 device id expected
    // @Description: The expected value of COMPASS_DEV_ID3, used by arming checks. Setting this to -1 means "don't care."
    // @User: Advanced
    AP_GROUPINFO("EXP_DID3", 38, Compass, _state[2].expected_dev_id, -1),
    
    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
Compass::Compass(void)
{
    if (_singleton != nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Compass must be singleton");
#endif
        return;
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// Default init method
//
bool
Compass::init()
{
    return true;
}

//  Register a new compass instance
//
uint8_t Compass::register_compass(void)
{
    return 0;
}

bool Compass::_add_backend(AP_Compass_Backend *backend)
{
    return true;
}

/*
  return true if a driver type is enabled
 */
bool Compass::_driver_enabled(enum DriverType driver_type)
{
    uint32_t mask = (1U<<uint8_t(driver_type));
    return (mask & uint32_t(_driver_type_mask.get())) == 0;
}

/*
  wrapper around hal.i2c_mgr->get_device() that prevents duplicate devices being opened
 */
bool Compass::_have_i2c_driver(uint8_t bus, uint8_t address) const
{
    return false;
}

/*
  macro to add a backend with check for too many backends or compass
  instances. We don't try to start more than the maximum allowed
 */
#define ADD_BACKEND(driver_type, backend)   \
    do { if (_driver_enabled(driver_type)) { _add_backend(backend); } \
       if (_backend_count == COMPASS_MAX_BACKEND || \
           _compass_count == COMPASS_MAX_INSTANCES) { \
          return; \
        } \
    } while (0)

#define GET_I2C_DEVICE(bus, address) _have_i2c_driver(bus, address)?nullptr:hal.i2c_mgr->get_device(bus, address)

/*
  look for compasses on external i2c buses
 */
void Compass::_probe_external_i2c_compasses(void)
{

}

/*
  detect available backends for this board
 */
void Compass::_detect_backends(void)
{
 
}

bool
Compass::read(void)
{
   return false;
}

uint8_t
Compass::get_healthy_mask() const
{
    return 0;
}

void
Compass::set_offsets(uint8_t i, const Vector3f &offsets)
{

}

void
Compass::set_and_save_offsets(uint8_t i, const Vector3f &offsets)
{

}

void
Compass::set_and_save_diagonals(uint8_t i, const Vector3f &diagonals)
{

}

void
Compass::set_and_save_offdiagonals(uint8_t i, const Vector3f &offdiagonals)
{

}

void
Compass::save_offsets(uint8_t i)
{

}

void
Compass::save_offsets(void)
{

}

void
Compass::set_motor_compensation(uint8_t i, const Vector3f &motor_comp_factor)
{

}

void
Compass::save_motor_compensation()
{

}

void
Compass::set_initial_location(int32_t latitude, int32_t longitude)
{

}

/// return true if the compass should be used for yaw calculations
bool
Compass::use_for_yaw(void) const
{
   return false;
}

/// return true if the specified compass can be used for yaw calculations
bool
Compass::use_for_yaw(uint8_t i) const
{
   return false;
}

void
Compass::set_declination(float radians, bool save_to_eeprom)
{

}

float
Compass::get_declination() const
{
   return 0;
}

/*
  calculate a compass heading given the attitude from DCM and the mag vector
 */
float
Compass::calculate_heading(const Matrix3f &dcm_matrix, uint8_t i) const
{
    return 0;
}

/// Returns True if the compasses have been configured (i.e. offsets saved)
///
/// @returns                    True if compass has been configured
///
bool Compass::configured(uint8_t i)
{
    return false;
}

bool Compass::configured(void)
{
    return false;
}

// Update raw magnetometer values from HIL data
//
void Compass::setHIL(uint8_t instance, float roll, float pitch, float yaw)
{

}

// Update raw magnetometer values from HIL mag vector
//
void Compass::setHIL(uint8_t instance, const Vector3f &mag, uint32_t update_usec)
{

}

const Vector3f& Compass::getHIL(uint8_t instance) const
{
    return _hil.field[instance];
}

// setup _Bearth
void Compass::_setup_earth_field(void)
{

}

/*
  set the type of motor compensation to use
 */
void Compass::motor_compensation_type(const uint8_t comp_type)
{

}

bool Compass::consistent() const
{
    return false;
}


// singleton instance
Compass *Compass::_singleton;

namespace AP {

Compass &compass()
{
    return *Compass::get_singleton();
}

}
