/*
 *  geo-fencing support
 *  Andrew Tridgell, December 2011
 */

#include "Plane.h"

#if GEOFENCE_ENABLED == ENABLED

#define MIN_GEOFENCE_POINTS 5 // index [0] for return point, must be inside polygon
                              // index [1 to n-1] to define a polygon, minimum 3 for a triangle
                              // index [n] (must be same as index 1 to close the polygon)

/*
 *  The state of geo-fencing. This structure is dynamically allocated
 *  the first time it is used. This means we only pay for the pointer
 *  and not the structure on systems where geo-fencing is not being
 *  used.
 *
 *  We store a copy of the boundary in memory as we need to access it
 *  very quickly at runtime
 */
static struct GeofenceState {
    uint8_t num_points;
    bool boundary_uptodate;
    bool fence_triggered;
    bool is_pwm_enabled;          //true if above FENCE_ENABLE_PWM threshold
    bool previous_is_pwm_enabled; //true if above FENCE_ENALBE_PWM threshold
                                  // last time we checked
    bool is_enabled;
    GeofenceEnableReason enable_reason;
    bool floor_enabled;          //typically used for landing
    uint16_t breach_count;
    uint8_t breach_type;
    uint32_t breach_time;
    uint8_t old_switch_position;
    int32_t guided_lat;
    int32_t guided_lng;
    /* point 0 is the return point */
    Vector2l *boundary;
} *geofence_state;


static const StorageAccess fence_storage(StorageManager::StorageFence);

/*
  maximum number of fencepoints
 */
uint8_t Plane::max_fencepoints(void)
{
    return 0;
}

/*
 *  fence boundaries fetch/store
 */
Vector2l Plane::get_fence_point_with_index(unsigned i)
{
    Vector2l ret;

    if (i > (unsigned)g.fence_total || i >= max_fencepoints()) {
        return Vector2l(0,0);
    }

    // read fence point
    ret.x = fence_storage.read_uint32(i * sizeof(Vector2l));
    ret.y = fence_storage.read_uint32(i * sizeof(Vector2l) + sizeof(int32_t));

    return ret;
}

// save a fence point
void Plane::set_fence_point_with_index(const Vector2l &point, unsigned i)
{

}

/*
 *  allocate and fill the geofence state structure
 */
void Plane::geofence_load(void)
{
 
}

/*
 *  Disable geofence and send an error message string
 */
void Plane::geofence_disable_and_send_error_msg(const char *errorMsg)
{

}

/*
 * return true if a geo-fence has been uploaded and
 * FENCE_ACTION is 1 (not necessarily enabled)
 */
bool Plane::geofence_present(void)
{
    return false;
}

/*
  check FENCE_CHANNEL and update the is_pwm_enabled state
 */
void Plane::geofence_update_pwm_enabled_state() 
{
  
}

//return true on success, false on failure
bool Plane::geofence_set_enabled(bool enable, GeofenceEnableReason r) 
{

    return true;
}

/*
 *  return true if geo-fencing is enabled
 */
bool Plane::geofence_enabled(void)
{
    return true;
}

/*
 * Set floor state IF the fence is present.
 * Return false on failure to set floor state.
 */
bool Plane::geofence_set_floor_enabled(bool floor_enable) {
    return true;
}

/*
 *  return true if we have breached the geo-fence minimum altiude
 */
bool Plane::geofence_check_minalt(void)
{
    return false;
}

/*
 *  return true if we have breached the geo-fence maximum altiude
 */
bool Plane::geofence_check_maxalt(void)
{
    if (g.fence_maxalt <= g.fence_minalt) {
        return false;
    }
    if (g.fence_maxalt == 0) {
        return false;
    }
    return (adjusted_altitude_cm() > (g.fence_maxalt*100.0f) + home.alt);
}


/*
 *  check if we have breached the geo-fence
 */
void Plane::geofence_check(bool altitude_check_only)
{
}

/*
 *  return true if geofencing allows stick mixing. When we have
 *  triggered failsafe and are in GUIDED mode then stick mixing is
 *  disabled. Otherwise the aircraft may not be able to recover from
 *  a breach of the geo-fence
 */
bool Plane::geofence_stickmixing(void) {
    if (geofence_enabled() &&
        geofence_state != nullptr &&
        geofence_state->fence_triggered &&
        (control_mode == GUIDED || control_mode == AVOID_ADSB)) {
        // don't mix in user input
        return false;
    }
    // normal mixing rules
    return true;
}

/*
 *
 */
void Plane::geofence_send_status(mavlink_channel_t chan)
{
    if (geofence_enabled() && geofence_state != nullptr) {
        mavlink_msg_fence_status_send(chan,
                                      (int8_t)geofence_state->fence_triggered,
                                      geofence_state->breach_count,
                                      geofence_state->breach_type,
                                      geofence_state->breach_time);
    }
}

/*
  return true if geofence has been breached
 */
bool Plane::geofence_breached(void)
{
    return geofence_state ? geofence_state->fence_triggered : false;
}


#else // GEOFENCE_ENABLED

void Plane::geofence_check(bool altitude_check_only) {
}
bool Plane::geofence_stickmixing(void) {
    return true;
}
bool Plane::geofence_enabled(void) {
    return false;
}

bool Plane::geofence_present(void) {
    return false;
}

bool Plane::geofence_set_enabled(bool enable, GeofenceEnableReason r) {
    return false;
}

bool Plane::geofence_set_floor_enabled(bool floor_enable) {
    return false;
}

bool Plane::geofence_breached(void)
{
    return false;
}

#endif // GEOFENCE_ENABLED
