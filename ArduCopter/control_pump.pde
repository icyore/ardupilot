/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static bool icy_pump_with_gps;

static uint32_t icy_pump_start_time;
static bool icy_pump_pause;

// pump_init - initialise pump controller
static bool pump_init(bool ignore_checks)
{
    // check if we have GPS and decide which pump we're going to do
    icy_pump_with_gps = GPS_ok();
    if (icy_pump_with_gps) {
        // set target to stopping point
        Vector3f stopping_point;
        wp_nav.get_loiter_stopping_point_xy(stopping_point);
        wp_nav.init_loiter_target(stopping_point);
    }

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(wp_nav.get_speed_down(), wp_nav.get_speed_up());
    pos_control.set_accel_z(wp_nav.get_accel_z());

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    icy_pump_start_time = millis();

    icy_pump_pause = false;

    return true;
}

// pump_run - runs the pump controller
// should be called at 100hz or more
static void pump_run()
{
        pump_nogps_run();
}


// pump_nogps_run - runs the pump controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
static void pump_nogps_run()
{
    int16_t target_roll = 0, target_pitch = 0;
    float target_yaw_rate = 0;

    // if not auto armed or pumped set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the pumping detector says we've pumped and throttle is at minimum
        if (ap.land_complete && (ap.throttle_zero || failsafe.radio)) {
            init_disarm_motors();
        }
#else
        // disarm when the pumping detector says we've pumped
        if (ap.land_complete) {
            init_disarm_motors();
        }
#endif
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // get pilot desired lean angles
            get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    //pause 4 seconds before beginning pump descent
    float cmb_rate;
    if(icy_pump_pause && millis()-icy_pump_start_time < LAND_WITH_DELAY_MS) {
        cmb_rate = 0;
    } else {
        icy_pump_pause = false;
        cmb_rate = get_throttle_pump();
    }

    // call position controller
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();
}

// get_throttle_pump - high level pumping logic
//      returns climb rate (in cm/s) which should be passed to the position controller
//      should be called at 100hz or higher
static float get_throttle_pump()
{
#if CONFIG_SONAR == ENABLED
    bool sonar_ok = sonar_enabled && sonar.healthy();
#else
    bool sonar_ok = false;
#endif
    // if we are above 10m and the sonar does not sense anything perform regular alt hold descent
    if (current_loc.alt >= LAND_START_ALT && !(sonar_ok && sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        return pos_control.get_speed_down();
    }else{
        return -abs(g.land_speed);
    }
}

// pump_do_not_use_GPS - forces pump-mode to not use the GPS but instead rely on pilot input for roll and pitch
//  called during GPS failsafe to ensure that if we were already in pump mode that we do not use the GPS
//  has no effect if we are not already in pump mode
static void pump_do_not_use_GPS()
{
    icy_pump_with_gps = false;
}

// set_mode_pump_with_pause - sets mode to pump and triggers 4 second delay before descent starts
static void set_mode_pump_with_pause()
{
    set_mode(PUMP);
    icy_pump_pause = true;
}

// pumping_with_GPS - returns true if vehicle is pumping using GPS
static bool pumping_with_GPS() {
    return (control_mode == PUMP && icy_pump_with_gps);
}
