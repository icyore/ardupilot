/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_newflightmode.pde - init and run calls for new flight mode
 */

// newflightmode_init - initialise flight mode
static bool newflightmode_init(bool ignore_checks)
{
    // do any required initialisation of the flight mode here
    // this code will be called whenever the operator switches into this mode

    // return true initialisation is successful, false if it fails
    // if false is returned here the vehicle will remain in the previous flight mode
    return true;
}

// newflightmode_run - runs the main controller
// will be called at 100hz or more
static void newflightmode_run()
{
    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || g.rc_3.control_in <= 0) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // convert pilot input into desired vehicle angles or rotation rates
    //   g.rc_1.control_in : pilots roll input in the range -4500 ~ 4500
    //   g.rc_2.control_in : pilot pitch input in the range -4500 ~ 4500
    //   g.rc_3.control_in : pilot's throttle input in the range 0 ~ 1000
    //   g.rc_4.control_in : pilot's yaw input in the range -4500 ~ 4500

    // call one of attitude controller's attitude control functions like
    //   attitude_control.angle_ef_roll_pitch_rate_yaw(roll angle, pitch angle, yaw rate);

    // call position controller's z-axis controller or simply pass through throttle
    //   attitude_control.set_throttle_out(desired throttle, true);
}
