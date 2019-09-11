#include "mode.h"
#include "Plane.h"
uint64_t t_in_fbwa;
uint64_t t0_fbwa = 0;
uint64_t t_last = 0;
uint64_t t_current;
float theta_sine;
float dt;
float wmin;
float wmax;
float Trec;
float K_sine;
float omega;
float d_aileron;
uint64_t chirp;

bool ModeFBWA::_enter()
{
    plane.throttle_allows_nudging = false;
    plane.auto_throttle_mode = false;
    plane.auto_navigation_mode = false;

    return true;
}

void ModeFBWA::update()
{
    // 08/22 Added Aileron chirp in FBWA mode
    if (t0_fbwa == 0) {
        // Set t0 so that the time in fbwa mode can be measured
        t0_fbwa = millis();
        theta_sine = 0;
    }
    t_in_fbwa = t0_fbwa; //ms
    t_in_fbwa = millis() - t_in_fbwa;
    t_current = millis(); //ms
    dt = t_current - t_last; //ms
    t_last = t_current; //ms

    if (t_in_fbwa < 25*1000) {
        chirp = 1;
        wmin = 3.14159; //rad/s
        wmax = 12*3.14159; //rad/s
        Trec = 25; //s
        if (t_in_fbwa < 1000*2*3.14159/(2*wmin)) { // First half period, 1 s
            K_sine = 0.0187*(expf(4*(t_in_fbwa/1000)/Trec)-1);
			omega = wmin;
			theta_sine += 0.001*omega*dt;
            d_aileron = 0.5*450*sinf(theta_sine);
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, d_aileron);
        }
        if (t_in_fbwa >= 1000*2*3.14159/(2*wmin)) {
            K_sine = 0.0187*(expf(4*(t_in_fbwa/1000)/Trec)-1);
			omega = wmin+K_sine*(wmax-wmin);
			theta_sine += omega*dt*0.001;
            d_aileron = 450*sinf(theta_sine);
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, d_aileron);
        }
    } // After 25s return to regular FBWA mode
    else {
        chirp = 0;
        // set nav_roll and nav_pitch using sticks
        plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
        plane.update_load_factor();
        float pitch_input = plane.channel_pitch->norm_input();
        if (pitch_input > 0) {
            plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max_cd;
        } else {
            plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min_cd);
        }
        plane.adjust_nav_pitch_throttle();
        plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min_cd, plane.aparm.pitch_limit_max_cd.get());
                if (plane.fly_inverted()) {
            plane.nav_pitch_cd = -plane.nav_pitch_cd;
        }
        if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
            // FBWA failsafe glide
            plane.nav_roll_cd = 0;
            plane.nav_pitch_cd = 0;
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_MIN);
        }
        if (plane.g.fbwa_tdrag_chan > 0) {
            // check for the user enabling FBWA taildrag takeoff mode
            bool tdrag_mode = (RC_Channels::get_radio_in(plane.g.fbwa_tdrag_chan-1) > 1700);
            if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
                if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
                    plane.auto_state.fbwa_tdrag_takeoff_mode = true;
                                        plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
                }
            }
        }
    }
    AP::logger().Write("CHRP","TimeUS,da-desired","Qf",
                       AP_HAL::micros64(),
                       d_aileron);
}
