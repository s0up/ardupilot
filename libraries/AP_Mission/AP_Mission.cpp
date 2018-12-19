/// @file    AP_Mission.cpp
/// @brief   Handles the MAVLINK command mission stack.  Reads and writes mission to storage.

#include "AP_Mission.h"
#include <AP_Terrain/AP_Terrain.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>

const AP_Param::GroupInfo AP_Mission::var_info[] = {

    // @Param: TOTAL
    // @DisplayName: Total mission commands
    // @Description: The number of mission mission items that has been loaded by the ground station. Do not change this manually.
    // @Range: 0 32766
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TOTAL",  0, AP_Mission, _cmd_total, 0),

    // @Param: RESTART
    // @DisplayName: Mission Restart when entering Auto mode
    // @Description: Controls mission starting point when entering Auto mode (either restart from beginning of mission or resume from last command run)
    // @Values: 0:Resume Mission, 1:Restart Mission
    // @User: Advanced
    AP_GROUPINFO("RESTART",  1, AP_Mission, _restart, AP_MISSION_RESTART_DEFAULT),

    // @Param: OPTIONS
    // @DisplayName: Mission options bitmask
    // @Description: Bitmask of what options to use in missions.
    // @Bitmask: 0:Clear Mission on reboot
    // @User: Advanced
    AP_GROUPINFO("OPTIONS",  2, AP_Mission, _options, AP_MISSION_OPTIONS_DEFAULT),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

// storage object
StorageAccess AP_Mission::_storage(StorageManager::StorageMission);

HAL_Semaphore_Recursive AP_Mission::_rsem;

///
/// public mission methods
///

/// init - initialises this library including checks the version in eeprom matches this library
void AP_Mission::init()
{

}

/// start - resets current commands to point to the beginning of the mission
///     To-Do: should we validate the mission first and return true/false?
void AP_Mission::start()
{

}

/// stop - stops mission execution.  subsequent calls to update() will have no effect until the mission is started or resumed
void AP_Mission::stop()
{

}

/// resume - continues the mission execution from where we last left off
///     previous running commands will be re-initialized
void AP_Mission::resume()
{

}

/// check mission starts with a takeoff command
bool AP_Mission::starts_with_takeoff_cmd()
{
    return false;
}

/// start_or_resume - if MIS_AUTORESTART=0 this will call resume(), otherwise it will call start()
void AP_Mission::start_or_resume()
{

}

/// reset - reset mission to the first command
void AP_Mission::reset()
{

}

/// clear - clears out mission
///     returns true if mission was running so it could not be cleared
bool AP_Mission::clear()
{
    return false;
}


/// trucate - truncate any mission items beyond index
void AP_Mission::truncate(uint16_t index)
{

}

/// update - ensures the command queues are loaded with the next command and calls main programs command_init and command_verify functions to progress the mission
///     should be called at 10hz or higher
void AP_Mission::update()
{

}

bool AP_Mission::verify_command(const Mission_Command& cmd)
{
    return false;
}

bool AP_Mission::start_command(const Mission_Command& cmd)
{
    return false;
}

///
/// public command methods
///

/// add_cmd - adds a command to the end of the command list and writes to storage
///     returns true if successfully added, false on failure
///     cmd.index is updated with it's new position in the mission
bool AP_Mission::add_cmd(Mission_Command& cmd)
{
    // attempt to write the command to storage
    return false;
}

/// replace_cmd - replaces the command at position 'index' in the command list with the provided cmd
///     replacing the current active command will have no effect until the command is restarted
///     returns true if successfully replaced, false on failure
bool AP_Mission::replace_cmd(uint16_t index, Mission_Command& cmd)
{
    return false;
}

/// is_nav_cmd - returns true if the command's id is a "navigation" command, false if "do" or "conditional" command
bool AP_Mission::is_nav_cmd(const Mission_Command& cmd)
{
    return true;
}

/// get_next_nav_cmd - gets next "navigation" command found at or after start_index
///     returns true if found, false if not found (i.e. reached end of mission command list)
///     accounts for do_jump commands but never increments the jump's num_times_run (advance_current_nav_cmd is responsible for this)
bool AP_Mission::get_next_nav_cmd(uint16_t start_index, Mission_Command& cmd)
{
    return false;
}

/// get the ground course of the next navigation leg in centidegrees
/// from 0 36000. Return default_angle if next navigation
/// leg cannot be determined
int32_t AP_Mission::get_next_ground_course_cd(int32_t default_angle)
{
    return 0;
}

// set_current_cmd - jumps to command specified by index
bool AP_Mission::set_current_cmd(uint16_t index)
{

    // if we got this far we must have successfully advanced the nav command
    return false;
}

/// load_cmd_from_storage - load command from storage
///     true is return if successful
bool AP_Mission::read_cmd_from_storage(uint16_t index, Mission_Command& cmd) const
{
    return false;
}

/// write_cmd_to_storage - write a command to storage
///     index is used to calculate the storage location
///     true is returned if successful
bool AP_Mission::write_cmd_to_storage(uint16_t index, Mission_Command& cmd)
{
    return false;
}

/// write_home_to_storage - writes the special purpose cmd 0 (home) to storage
///     home is taken directly from ahrs
void AP_Mission::write_home_to_storage()
{

}

MAV_MISSION_RESULT AP_Mission::sanity_check_params(const mavlink_mission_item_int_t& packet) {
    uint8_t nan_mask;
    switch (packet.command) {
        case MAV_CMD_NAV_WAYPOINT:
            nan_mask = ~(1 << 3); // param 4 can be nan
            break;
        case MAV_CMD_NAV_LAND:
            nan_mask = ~(1 << 3); // param 4 can be nan
            break;
        case MAV_CMD_NAV_TAKEOFF:
            nan_mask = ~(1 << 3); // param 4 can be nan
            break;
        case MAV_CMD_NAV_VTOL_TAKEOFF:
            nan_mask = ~(1 << 3); // param 4 can be nan
            break;
        case MAV_CMD_NAV_VTOL_LAND:
            nan_mask = ~((1 << 2) | (1 << 3)); // param 3 and 4 can be nan
            break;
        default:
            nan_mask = 0xff;
            break;
    }

    if (((nan_mask & (1 << 0)) && isnan(packet.param1)) ||
        isinf(packet.param1)) {
        return MAV_MISSION_INVALID_PARAM1;
    }
    if (((nan_mask & (1 << 1)) && isnan(packet.param2)) ||
        isinf(packet.param2)) {
        return MAV_MISSION_INVALID_PARAM2;
    }
    if (((nan_mask & (1 << 2)) && isnan(packet.param3)) ||
        isinf(packet.param3)) {
        return MAV_MISSION_INVALID_PARAM3;
    }
    if (((nan_mask & (1 << 3)) && isnan(packet.param4)) ||
        isinf(packet.param4)) {
        return MAV_MISSION_INVALID_PARAM4;
    }
    return MAV_MISSION_ACCEPTED;
}

// mavlink_to_mission_cmd - converts mavlink message to an AP_Mission::Mission_Command object which can be stored to eeprom
//  return MAV_MISSION_ACCEPTED on success, MAV_MISSION_RESULT error on failure
MAV_MISSION_RESULT AP_Mission::mavlink_int_to_mission_cmd(const mavlink_mission_item_int_t& packet, AP_Mission::Mission_Command& cmd)
{
    // if we got this far then it must have been successful
    return MAV_MISSION_ACCEPTED;
}

// converts a mission_item to mission_item_int and returns a mission_command
MAV_MISSION_RESULT AP_Mission::mavlink_to_mission_cmd(const mavlink_mission_item_t& packet, AP_Mission::Mission_Command& cmd)
{
    return MAV_MISSION_ACCEPTED;
}

// converts a Mission_Command to mission_item_int and returns a mission_item
bool AP_Mission::mission_cmd_to_mavlink(const AP_Mission::Mission_Command& cmd, mavlink_mission_item_t& packet)
{
    return false;
}

// mavlink_cmd_long_to_mission_cmd - converts a mavlink cmd long to an AP_Mission::Mission_Command object which can be stored to eeprom
// return MAV_MISSION_ACCEPTED on success, MAV_MISSION_RESULT error on failure
MAV_MISSION_RESULT AP_Mission::mavlink_cmd_long_to_mission_cmd(const mavlink_command_long_t& packet, AP_Mission::Mission_Command& cmd) 
{
    return MAV_MISSION_ACCEPTED;
}

// mission_cmd_to_mavlink - converts an AP_Mission::Mission_Command object to a mavlink message which can be sent to the GCS
//  return true on success, false on failure
bool AP_Mission::mission_cmd_to_mavlink_int(const AP_Mission::Mission_Command& cmd, mavlink_mission_item_int_t& packet)
{
    return false;
}

///
/// private methods
///

/// complete - mission is marked complete and clean-up performed including calling the mission_complete_fn
void AP_Mission::complete()
{

}

/// advance_current_nav_cmd - moves current nav command forward
///     do command will also be loaded
///     accounts for do-jump commands
//      returns true if command is advanced, false if failed (i.e. mission completed)
bool AP_Mission::advance_current_nav_cmd(uint16_t starting_index)
{
    return false;
}

/// advance_current_do_cmd - moves current do command forward
///     accounts for do-jump commands
void AP_Mission::advance_current_do_cmd()
{
 
}

/// get_next_cmd - gets next command found at or after start_index
///     returns true if found, false if not found (i.e. mission complete)
///     accounts for do_jump commands
///     increment_jump_num_times_if_found should be set to true if advancing the active navigation command
bool AP_Mission::get_next_cmd(uint16_t start_index, Mission_Command& cmd, bool increment_jump_num_times_if_found)
{

    return false;
}

/// get_next_do_cmd - gets next "do" or "conditional" command after start_index
///     returns true if found, false if not found
///     stops and returns false if it hits another navigation command before it finds the first do or conditional command
///     accounts for do_jump commands but never increments the jump's num_times_run (advance_current_nav_cmd is responsible for this)
bool AP_Mission::get_next_do_cmd(uint16_t start_index, Mission_Command& cmd)
{
    return false;
}

///
/// jump handling methods
///

// init_jump_tracking - initialise jump_tracking variables
void AP_Mission::init_jump_tracking()
{

}

/// get_jump_times_run - returns number of times the jump command has been run
int16_t AP_Mission::get_jump_times_run(const Mission_Command& cmd)
{
    return AP_MISSION_JUMP_TIMES_MAX;
}

/// increment_jump_times_run - increments the recorded number of times the jump command has been run
void AP_Mission::increment_jump_times_run(Mission_Command& cmd)
{

}

// check_eeprom_version - checks version of missions stored in eeprom matches this library
// command list will be cleared if they do not match
void AP_Mission::check_eeprom_version()
{

}

/*
  return total number of commands that can fit in storage space
 */
uint16_t AP_Mission::num_commands_max(void) const
{
    // -4 to remove space for eeprom version number
    return (_storage.size() - 4) / AP_MISSION_EEPROM_COMMAND_SIZE;
}

// find the nearest landing sequence starting point (DO_LAND_START) and
// return its index.  Returns 0 if no appropriate DO_LAND_START point can
// be found.
uint16_t AP_Mission::get_landing_sequence_start() 
{
    return 0;
}

/*
   find the nearest landing sequence starting point (DO_LAND_START) and
   switch to that mission item.  Returns false if no DO_LAND_START
   available.
 */
bool AP_Mission::jump_to_landing_sequence(void)
{
    return false;
}

// jumps the mission to the closest landing abort that is planned, returns false if unable to find a valid abort
bool AP_Mission::jump_to_abort_landing_sequence(void)
{
    return false;
}

const char *AP_Mission::Mission_Command::type() const {
    switch(id) {
    case MAV_CMD_NAV_WAYPOINT:
        return "WP";
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return "RTL";
    case MAV_CMD_NAV_LOITER_UNLIM:
        return "LoitUnlim";
    case MAV_CMD_NAV_LOITER_TIME:
        return "LoitTime";
    case MAV_CMD_NAV_SET_YAW_SPEED:
        return "SetYawSpd";
    case MAV_CMD_CONDITION_DELAY:
        return "CondDelay";
    case MAV_CMD_CONDITION_DISTANCE:
        return "CondDist";
    case MAV_CMD_DO_CHANGE_SPEED:
        return "ChangeSpeed";
    case MAV_CMD_DO_SET_HOME:
        return "SetHome";
    case MAV_CMD_DO_SET_SERVO:
        return "SetServo";
    case MAV_CMD_DO_SET_RELAY:
        return "SetRelay";
    case MAV_CMD_DO_REPEAT_SERVO:
        return "RepeatServo";
    case MAV_CMD_DO_REPEAT_RELAY:
        return "RepeatRelay";
    case MAV_CMD_DO_CONTROL_VIDEO:
        return "CtrlVideo";
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
        return "DigiCamCfg";
    case MAV_CMD_DO_DIGICAM_CONTROL:
        return "DigiCamCtrl";
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        return "SetCamTrigDst";
    case MAV_CMD_DO_SET_ROI:
        return "SetROI";
    case MAV_CMD_DO_SET_REVERSE:
        return "SetReverse";
    default:
        return "?";
    }
}

// singleton instance
AP_Mission *AP_Mission::_singleton;

namespace AP {

AP_Mission *mission()
{
    return AP_Mission::get_singleton();
}

}
