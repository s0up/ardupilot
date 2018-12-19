#include "AP_Tuning.h"
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Tuning::var_info[] = {
    // @Param: CHAN
    // @DisplayName: Transmitter tuning channel
    // @Description: This sets the channel for transmitter tuning. This should be connected to a knob or slider on your transmitter. It needs to be setup to use the PWM range given by TUNE_CHAN_MIN to TUNE_CHAN_MAX
    // @Values: 0:Disable,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    // @User: Standard
    AP_GROUPINFO("CHAN", 1, AP_Tuning, channel, 0),
    
    // @Param: CHAN_MIN
    // @DisplayName: Transmitter tuning channel minimum pwm
    // @Description: This sets the PWM lower limit for the tuning channel
    // @Range: 900 2100
    // @User: Standard
    AP_GROUPINFO("CHAN_MIN", 2, AP_Tuning, channel_min, 1000),

    // @Param: CHAN_MAX
    // @DisplayName: Transmitter tuning channel maximum pwm
    // @Description: This sets the PWM upper limit for the tuning channel
    // @Range: 900 2100
    // @User: Standard
    AP_GROUPINFO("CHAN_MAX", 3, AP_Tuning, channel_max, 2000),

    // @Param: SELECTOR
    // @DisplayName: Transmitter tuning selector channel
    // @Description: This sets the channel for the transmitter tuning selector switch. This should be a 2 position switch, preferably spring loaded. A PWM above 1700 means high, below 1300 means low. If no selector is set then you won't be able to switch between parameters during flight or re-center the tuning knob
    // @Values: 0:Disable,1:Chan1,2:Chan3,3:Chan3,4:Chan4,5:Chan5,6:Chan6,7:Chan7,8:Chan8,9:Chan9,10:Chan10,11:Chan11,12:Chan12,13:Chan13,14:Chan14,15:Chan15,16:Chan16
    // @User: Standard
    AP_GROUPINFO("SELECTOR", 4, AP_Tuning, selector, 0),
    
    // @Param: RANGE
    // @DisplayName: Transmitter tuning range
    // @Description: This sets the range over which tuning will change a parameter. A value of 2 means the tuning parameter will go from 0.5 times the start value to 2x the start value over the range of the tuning channel
    // @User: Standard
    AP_GROUPINFO("RANGE", 5, AP_Tuning, range, 2.0f),

    // @Param: MODE_REVERT
    // @DisplayName: Revert on mode change
    // @Description: This controls whether tuning values will revert on a flight mode change.
    // @Values: 0:Disable,1:Enable
    // @User: Standard
    AP_GROUPINFO("MODE_REVERT", 6, AP_Tuning, mode_revert, 1),

    // @Param: ERR_THRESH
    // @DisplayName: Controller error threshold
    // @Description: This sets the controller error threshold above which an alarm will sound and a message will be sent to the GCS to warn of controller instability
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("ERR_THRESH", 7, AP_Tuning, error_threshold, 0.15f),
    
    AP_GROUPEND
};

/*
  handle selector switch input
*/
void AP_Tuning::check_selector_switch(void)
{

}

/*
  re-center the tuning value
 */
void AP_Tuning::re_center(void)
{

}

/*
  check for changed tuning input
 */
void AP_Tuning::check_input(uint8_t flightmode)
{
 
}


/*
  log a tuning change
 */
void AP_Tuning::Log_Write_Parameter_Tuning(float value)
{

}

/*
  save parameters in the set
 */
void AP_Tuning::save_parameters(void)
{

}


/*
  save parameters in the set
 */
void AP_Tuning::revert_parameters(void)
{

}

/*
  switch to the next parameter in the set
 */
void AP_Tuning::next_parameter(void)
{

}

/*
  return a string representing a tuning parameter
 */
const char *AP_Tuning::get_tuning_name(uint8_t parm)
{
    return nullptr;
}

/*
  check for controller error
 */
void AP_Tuning::check_controller_error(void)
{

}
