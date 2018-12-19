#include "AP_Camera.h"
#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_HAL/AP_HAL.h>

// ------------------------------
#define CAM_DEBUG DISABLED

const AP_Param::GroupInfo AP_Camera::var_info[] = {
    // @Param: TRIGG_TYPE
    // @DisplayName: Camera shutter (trigger) type
    // @Description: how to trigger the camera to take a picture
    // @Values: 0:Servo,1:Relay
    // @User: Standard
    AP_GROUPINFO("TRIGG_TYPE",  0, AP_Camera, _trigger_type, AP_CAMERA_TRIGGER_DEFAULT_TRIGGER_TYPE),

    // @Param: DURATION
    // @DisplayName: Duration that shutter is held open
    // @Description: How long the shutter will be held open in 10ths of a second (i.e. enter 10 for 1second, 50 for 5seconds)
    // @Units: ds
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("DURATION",    1, AP_Camera, _trigger_duration, AP_CAMERA_TRIGGER_DEFAULT_DURATION),

    // @Param: SERVO_ON
    // @DisplayName: Servo ON PWM value
    // @Description: PWM value in microseconds to move servo to when shutter is activated
    // @Units: PWM
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SERVO_ON",    2, AP_Camera, _servo_on_pwm, AP_CAMERA_SERVO_ON_PWM),

    // @Param: SERVO_OFF
    // @DisplayName: Servo OFF PWM value
    // @Description: PWM value in microseconds to move servo to when shutter is deactivated
    // @Units: PWM
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SERVO_OFF",   3, AP_Camera, _servo_off_pwm, AP_CAMERA_SERVO_OFF_PWM),

    // @Param: TRIGG_DIST
    // @DisplayName: Camera trigger distance
    // @Description: Distance in meters between camera triggers. If this value is non-zero then the camera will trigger whenever the GPS position changes by this number of meters regardless of what mode the APM is in. Note that this parameter can also be set in an auto mission using the DO_SET_CAM_TRIGG_DIST command, allowing you to enable/disable the triggering of the camera during the flight.
    // @User: Standard
    // @Units: m
    // @Range: 0 1000
    AP_GROUPINFO("TRIGG_DIST",  4, AP_Camera, _trigg_dist, 0),

    // @Param: RELAY_ON
    // @DisplayName: Relay ON value
    // @Description: This sets whether the relay goes high or low when it triggers. Note that you should also set RELAY_DEFAULT appropriately for your camera
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("RELAY_ON",    5, AP_Camera, _relay_on, 1),

    // @Param: MIN_INTERVAL
    // @DisplayName: Minimum time between photos
    // @Description: Postpone shooting if previous picture was taken less than preset time(ms) ago.
    // @User: Standard
    // @Units: ms
    // @Range: 0 10000
    AP_GROUPINFO("MIN_INTERVAL",  6, AP_Camera, _min_interval, 0),

    // @Param: MAX_ROLL
    // @DisplayName: Maximum photo roll angle.
    // @Description: Postpone shooting if roll is greater than limit. (0=Disable, will shoot regardless of roll).
    // @User: Standard
    // @Units: deg
    // @Range: 0 180
    AP_GROUPINFO("MAX_ROLL",  7, AP_Camera, _max_roll, 0),

    // @Param: FEEDBACK_PIN
    // @DisplayName: Camera feedback pin
    // @Description: pin number to use for save accurate camera feedback messages. If set to -1 then don't use a pin flag for this, otherwise this is a pin number which if held high after a picture trigger order, will save camera messages when camera really takes a picture. A universal camera hot shoe is needed. The pin should be held high for at least 2 milliseconds for reliable trigger detection. See also the CAM_FEEDBACK_POL option.
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("FEEDBACK_PIN",  8, AP_Camera, _feedback_pin, AP_CAMERA_FEEDBACK_DEFAULT_FEEDBACK_PIN),

    // @Param: FEEDBACK_POL
    // @DisplayName: Camera feedback pin polarity
    // @Description: Polarity for feedback pin. If this is 1 then the feedback pin should go high on trigger. If set to 0 then it should go low
    // @Values: 0:TriggerLow,1:TriggerHigh
    // @User: Standard
    AP_GROUPINFO("FEEDBACK_POL",  9, AP_Camera, _feedback_polarity, 1),

    // @Param: AUTO_ONLY
    // @DisplayName: Distance-trigging in AUTO mode only
    // @Description: When enabled, trigging by distance is done in AUTO mode only.
    // @Values: 0:Always,1:Only when in AUTO
    // @User: Standard
    AP_GROUPINFO("AUTO_ONLY",  10, AP_Camera, _auto_mode_only, 0),

    // @Param: TYPE
    // @DisplayName: Type of camera (0: None, 1: BMMCC)
    // @Description: Set the camera type that is being used, certain cameras have custom functions that need further configuration, this enables that.
    // @Values: 0:Default,1:BMMCC
    // @User: Standard
    AP_GROUPINFO("TYPE",  11, AP_Camera, _type, 0),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

/// Servo operated camera
void
AP_Camera::servo_pic()
{

}

/// basic relay activation
void
AP_Camera::relay_pic()
{

}

/// single entry point to take pictures
///  set send_mavlink_msg to true to send DO_DIGICAM_CONTROL message to all components
void AP_Camera::trigger_pic()
{

}

/// de-activate the trigger after some delay, but without using a delay() function
/// should be called at 50hz
void
AP_Camera::trigger_pic_cleanup()
{

}

/// decode deprecated MavLink message that controls camera.
void
AP_Camera::control_msg(const mavlink_message_t* msg)
{

}

void AP_Camera::configure(float shooting_mode, float shutter_speed, float aperture, float ISO, float exposure_type, float cmd_id, float engine_cutoff_time)
{

}

void AP_Camera::control(float session, float zoom_pos, float zoom_step, float focus_lock, float shooting_cmd, float cmd_id)
{

}

/*
  Send camera feedback to the GCS
 */
void AP_Camera::send_feedback(mavlink_channel_t chan)
{

}


/*  update; triggers by distance moved
*/
void AP_Camera::update()
{

}

/*
  interrupt handler for interrupt based feedback trigger
 */
void AP_Camera::feedback_pin_isr(uint8_t pin, bool high, uint32_t timestamp_us)
{

}

/*
  check if feedback pin is high for timer based feedback trigger, when
  attach_interrupt fails
 */
void AP_Camera::feedback_pin_timer(void)
{

}

/*
  setup a callback for a feedback pin. When on PX4 with the right FMU
  mode we can use the microsecond timer.
 */
void AP_Camera::setup_feedback_callback(void)
{

}

// log_picture - log picture taken and send feedback to GCS
void AP_Camera::log_picture()
{

}

// take_picture - take a picture
void AP_Camera::take_picture()
{

}

/*
  update camera trigger - 50Hz
 */
void AP_Camera::update_trigger()
{

}

// singleton instance
AP_Camera *AP_Camera::_singleton;

namespace AP {

AP_Camera *camera()
{
    return AP_Camera::get_singleton();
}

}
