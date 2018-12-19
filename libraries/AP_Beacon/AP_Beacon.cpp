/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Beacon.h"
#include "AP_Beacon_Backend.h"
#include "AP_Beacon_Pozyx.h"
#include "AP_Beacon_Marvelmind.h"
#include "AP_Beacon_SITL.h"

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Beacon::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Beacon based position estimation device type
    // @Description: What type of beacon based position estimation device is connected
    // @Values: 0:None,1:Pozyx,2:Marvelmind
    // @User: Advanced
    AP_GROUPINFO("_TYPE",    0, AP_Beacon, _type, 0),

    // @Param: _LATITUDE
    // @DisplayName: Beacon origin's latitude
    // @Description: Beacon origin's latitude
    // @Units: deg
    // @Increment: 0.000001
    // @Range: -90 90
    // @User: Advanced
    AP_GROUPINFO("_LATITUDE", 1, AP_Beacon, origin_lat, 0),

    // @Param: _LONGITUDE
    // @DisplayName: Beacon origin's longitude
    // @Description: Beacon origin's longitude
    // @Units: deg
    // @Increment: 0.000001
    // @Range: -180 180
    // @User: Advanced
    AP_GROUPINFO("_LONGITUDE", 2, AP_Beacon, origin_lon, 0),

    // @Param: _ALT
    // @DisplayName: Beacon origin's altitude above sealevel in meters
    // @Description: Beacon origin's altitude above sealevel in meters
    // @Units: m
    // @Increment: 1
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("_ALT", 3, AP_Beacon, origin_alt, 0),

    // @Param: _ORIENT_YAW
    // @DisplayName: Beacon systems rotation from north in degrees
    // @Description: Beacon systems rotation from north in degrees
    // @Units: deg
    // @Increment: 1
    // @Range: -180 +180
    // @User: Advanced
    AP_GROUPINFO("_ORIENT_YAW", 4, AP_Beacon, orient_yaw, 0),

    AP_GROUPEND
};

AP_Beacon::AP_Beacon(AP_SerialManager &_serial_manager) :
    serial_manager(_serial_manager)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise the AP_Beacon class
void AP_Beacon::init(void)
{

}

// return true if beacon feature is enabled
bool AP_Beacon::enabled(void)
{
    return false;
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon::healthy(void)
{
    return false;
}

// update state. This should be called often from the main loop
void AP_Beacon::update(void)
{

}

// return origin of position estimate system
bool AP_Beacon::get_origin(Location &origin_loc) const
{
    return false;
}

// return position in NED from position estimate system's origin in meters
bool AP_Beacon::get_vehicle_position_ned(Vector3f &position, float& accuracy_estimate) const
{
    return false;
}

// return the number of beacons
uint8_t AP_Beacon::count() const
{
    return 0;
}

// return all beacon data
bool AP_Beacon::get_beacon_data(uint8_t beacon_instance, struct BeaconState& state) const
{
   return false;
}

// return individual beacon's id
uint8_t AP_Beacon::beacon_id(uint8_t beacon_instance) const
{
    return 0;
}

// return beacon health
bool AP_Beacon::beacon_healthy(uint8_t beacon_instance) const
{
    return false;
}

// return distance to beacon in meters
float AP_Beacon::beacon_distance(uint8_t beacon_instance) const
{
    return 0.0f;
}

// return beacon position in meters
Vector3f AP_Beacon::beacon_position(uint8_t beacon_instance) const
{
    Vector3f temp = {};
    return temp;
}

// return last update time from beacon in milliseconds
uint32_t AP_Beacon::beacon_last_update_ms(uint8_t beacon_instance) const
{
   return 0;
}

// create fence boundary points
void AP_Beacon::update_boundary_points()
{

}

// find next boundary point from an array of boundary points given the current index into that array
// returns true if a next point can be found
//   current_index should be an index into the boundary_pts array
//   start_angle is an angle (in radians), the search will sweep clockwise from this angle
//   the index of the next point is returned in the next_index argument
//   the angle to the next point is returned in the next_angle argument
bool AP_Beacon::get_next_boundary_point(const Vector2f* boundary_pts, uint8_t num_points, uint8_t current_index, float start_angle, uint8_t& next_index, float& next_angle)
{
    return false;
}

// return fence boundary array
const Vector2f* AP_Beacon::get_boundary_points(uint16_t& num_points) const
{
    num_points = 0;
    return nullptr;
}

// check if the device is ready
bool AP_Beacon::device_ready(void) const
{
    return false;
}
