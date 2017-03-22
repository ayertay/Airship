// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

/*
 *       AP_MotorsSingle.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsAirship.h"

extern const AP_HAL::HAL& hal;


const AP_Param::GroupInfo AP_MotorsAirship::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    // parameters 1 ~ 29 were reserved for tradheli
    // parameters 30 ~ 39 reserved for tricopter
    // parameters 40 ~ 49 for single copter and coax copter (these have identical parameter files)

    // 40 was ROLL_SV_REV
    // 41 was PITCH_SV_REV
    // 42 was YAW_SV_REV

	// @Param: SV_SPEED
    // @DisplayName: Servo speed 
    // @Description: Servo update speed
    // @Units: Hz
    AP_GROUPINFO("SV_SPEED", 43, AP_MotorsAirship, _servo_speed, AP_MOTORS_SINGLE_SPEED_ANALOG_SERVOS),

    // @Group: SV1_
    // @Path: ../RC_Channel/RC_Channel.cpp
    AP_SUBGROUPINFO(_servo1, "SV1_", 44, AP_MotorsAirship, RC_Channel),
    // @Group: SV2_
    // @Path: ../RC_Channel/RC_Channel.cpp
    AP_SUBGROUPINFO(_servo2, "SV2_", 45, AP_MotorsAirship, RC_Channel),
    // @Group: SV3_
    // @Path: ../RC_Channel/RC_Channel.cpp
    AP_SUBGROUPINFO(_servo3, "SV3_", 46, AP_MotorsAirship, RC_Channel),
    // @Group: SV4_
    // @Path: ../RC_Channel/RC_Channel.cpp
    AP_SUBGROUPINFO(_servo4, "SV4_", 47, AP_MotorsAirship, RC_Channel),

    AP_GROUPEND
};
// init
void AP_MotorsAirship::Init()
{
    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the main ESC can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;
    motor_enabled[AP_MOTORS_MOT_7] = true;

    // we set four servos to angle
    _servo1.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo2.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo3.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo4.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo1.set_angle(AP_MOTORS_AIRSHIP_SERVO_INPUT_RANGE);
    _servo2.set_angle(AP_MOTORS_AIRSHIP_SERVO_INPUT_RANGE);
    _servo3.set_angle(AP_MOTORS_AIRSHIP_SERVO_INPUT_RANGE);
    _servo4.set_angle(AP_MOTORS_AIRSHIP_SERVO_INPUT_RANGE);
}

// set update rate to motors - a value in hertz
void AP_MotorsAirship::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 4 servos and 3 motors
    uint32_t mask =
        1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3 |
        1U << AP_MOTORS_MOT_5 |
        1U << AP_MOTORS_MOT_6 ;
    rc_set_freq(mask, _servo_speed);
    uint32_t mask2 =
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_4 |
        1U << AP_MOTORS_MOT_7 ;
    rc_set_freq(mask2, _speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsAirship::enable()
{
    // enable output channels
    rc_enable_ch(AP_MOTORS_MOT_1);
    rc_enable_ch(AP_MOTORS_MOT_2);
    rc_enable_ch(AP_MOTORS_MOT_3);
    rc_enable_ch(AP_MOTORS_MOT_4);
    rc_enable_ch(AP_MOTORS_MOT_5);
    rc_enable_ch(AP_MOTORS_MOT_6);
    rc_enable_ch(AP_MOTORS_MOT_7);
}

void AP_MotorsAirship::output_to_motors()
{
    // Motors 1, 4, 7
    // Servo  2, 3, 5, 6
    switch (_spool_mode) {
        case SHUT_DOWN:
            // sends minimum values out to the motors
            hal.rcout->cork();
            rc_write(AP_MOTORS_MOT_2, calc_pwm_output_1to1(_roll_radio_passthrough + _yaw_radio_passthrough, _servo1)); //## Should probably be different
            rc_write(AP_MOTORS_MOT_3, calc_pwm_output_1to1(_roll_radio_passthrough + _yaw_radio_passthrough, _servo2));
            rc_write(AP_MOTORS_MOT_5, calc_pwm_output_1to1(-_roll_radio_passthrough +  _yaw_radio_passthrough, _servo3));
            rc_write(AP_MOTORS_MOT_6, calc_pwm_output_1to1(-_roll_radio_passthrough + _yaw_radio_passthrough, _servo4));
            rc_write(AP_MOTORS_MOT_1, get_pwm_output_min());
            rc_write(AP_MOTORS_MOT_4, get_pwm_output_min());
            rc_write(AP_MOTORS_MOT_7, get_pwm_output_min());
            hal.rcout->push();
            break;
        case SPIN_WHEN_ARMED:
            // sends output to motors when armed but not flying
            hal.rcout->cork();
            rc_write(AP_MOTORS_MOT_2, calc_pwm_output_1to1(_spin_up_ratio * _actuator_out[0], _servo1));
            rc_write(AP_MOTORS_MOT_3, calc_pwm_output_1to1(_spin_up_ratio * _actuator_out[1], _servo2));
            rc_write(AP_MOTORS_MOT_5, calc_pwm_output_1to1(_spin_up_ratio * _actuator_out[2], _servo3));
            rc_write(AP_MOTORS_MOT_6, calc_pwm_output_1to1(_spin_up_ratio * _actuator_out[3], _servo4));
            rc_write(AP_MOTORS_MOT_1, calc_spin_up_to_pwm());
            rc_write(AP_MOTORS_MOT_4, calc_spin_up_to_pwm());
            rc_write(AP_MOTORS_MOT_7, calc_spin_up_to_pwm());
            hal.rcout->push();
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
        case SPOOL_DOWN:
            // set motor output based on thrust requests
            hal.rcout->cork();






            rc_write(AP_MOTORS_MOT_2, calc_pwm_output_1to1(_actuator_out[0], _servo1));
            rc_write(AP_MOTORS_MOT_3, calc_pwm_output_1to1(_actuator_out[1], _servo2));
            rc_write(AP_MOTORS_MOT_5, calc_pwm_output_1to1(_actuator_out[2], _servo3));
            rc_write(AP_MOTORS_MOT_6, calc_pwm_output_1to1(_actuator_out[3], _servo4));
            rc_write(AP_MOTORS_MOT_1, calc_thrust_to_pwm(_rpm_L));
            rc_write(AP_MOTORS_MOT_4, calc_thrust_to_pwm(_rpm_R));
            rc_write(AP_MOTORS_MOT_7, calc_thrust_to_pwm(_thrust_T)); 
            hal.rcout->push();
            break;
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsAirship::get_motor_mask()
{
    uint32_t mask =
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3 |
        1U << AP_MOTORS_MOT_4 |
        1U << AP_MOTORS_MOT_5 |
        1U << AP_MOTORS_MOT_6 |
        1U << AP_MOTORS_MOT_7;
    return rc_map_mask(mask);
}

// sends commands to the motors
    //## These are the variables to set for output
/*    _actuator_out[NUM_ACTUATORS]; // Servo outputs for the two motors 0,1 is on left rotor and 2,3 on the right
    _rpm_L; // Left rotor
    _rpm_R; // Right rotor
    _thrust_T;  // Top pitch control prop
*/
void AP_MotorsAirship::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, 0 - 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   forward_thrust;             //## forward thurst input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, +/- 1.0
    float   throttle_left;            //##Increasese RPM, linked to CH8, 0 - 1.0 
    float   throttle_right;              //## control RPM,  linked to CH7 , 0 - 1.0
    //float pitch_helper;
    //float   thrust_min_rpy;             // the minimum throttle setting that will not limit the roll and pitch output
    //float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy
    //float   thrust_out;                 //
    //float   rp_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
    //float   actuator_allowed = 0.0f;    // amount of yaw we can fit in

    //## rotation of forward/thrust to dx/dy by alpha degrees
    float   angle_cos = 0.71;           //## cos component of cos(alpha)
    float   angle_sin = 0.71;           //## sin(alpha)
    float k1 = 0.135;
    float k2 = 0.165;

    // apply voltage and air pressure compensation
    roll_thrust = read_ch(1)*2.0f - 1.0f;
    pitch_thrust = read_ch(2)*2.0f - 1.0f;
    yaw_thrust = read_ch(4)*2.0f - 1.0f;
    throttle_thrust = read_ch(3);
    //roll_thrust = _roll_in * get_compensation_gain();
    //pitch_thrust = _pitch_in * get_compensation_gain();
    //yaw_thrust = _yaw_in * get_compensation_gain();
    forward_thrust = read_ch(5)*2.0f - 1.0f;
    //throttle_thrust = get_throttle() * get_compensation_gain()/2.0f; //##Consider changing!! 
    forward_thrust = constrain_float(forward_thrust, -1.0f, 1.0f);
    throttle_left = read_ch(7);
    throttle_left = constrain_float(throttle_left, 0.0f, 1.0f);
    throttle_right = read_ch(6);
    throttle_right = constrain_float(throttle_right, 0.0f, 1.0f);
    // Begin with mapping pitch control thrust 0-1, the constant 0.5 shoud be a close guess to normal pitch correction.
    _thrust_T = pitch_thrust/2.0f + 0.5;

    if(_thrust_T <= 0.0f){
        _thrust_T = 0.0f;
        // Do something to tell the controller that pitch is saturated.
        limit.roll_pitch = true;
    }
    else if (_thrust_T > 1.0f)
    {
        _thrust_T = 1.0f;
        // Pitch control is saturated.
        limit.roll_pitch = true;
    }



    //## Check so that the total thrust (lift) and forward thrust doesn't exced the maximum thrust the rotors can produce.
    if (throttle_thrust*throttle_thrust + forward_thrust*forward_thrust >= _throttle_thrust_max) {
        //throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // Adding compenstation for pitch when we have a better understading of the rotors
    //  in these would improve accuracy but skip until later
    // Left rotor servos
   // _actuator_out[0] = (forward_thrust + yaw_thrust) * angle_cos - (throttle_thrust - roll_thrust)* angle_sin; 
      _actuator_out[0] = throttle_thrust + (roll_thrust  + 0.5f*yaw_thrust) / 7.0f; //delX
   // _actuator_out[1] = (forward_thrust + yaw_thrust) * angle_sin + (throttle_thrust - roll_thrust)* angle_cos;
      _actuator_out[1] = forward_thrust  + (0.5f*roll_thrust + yaw_thrust ) / 7.0f; //delY
    // Right rotor servos
    //_actuator_out[2] = (forward_thrust - yaw_thrust) * angle_cos - (throttle_thrust + roll_thrust)* angle_sin;
      _actuator_out[2] = throttle_thrust - roll_thrust / 7.0f + 0.5f*yaw_thrust / 7.0f; //delX
      _actuator_out[3] = -forward_thrust - 0.5f*roll_thrust / 7.0f + yaw_thrust / 7.0f; //delY
    //_actuator_out[3] = (forward_thrust - yaw_thrust) * angle_sin + (throttle_thrust + roll_thrust)* angle_cos; 
    _rpm_L = throttle_left;// + abs(_actuator_out[0])*k1/(1 + throttle_offset) + abs(_actuator_out[1])*k2/(1 + throttle_offset);//get_rotor_rpm(_rpm_L, _actuator_out[0], _actuator_out[1]); // Left rotor motor
    _rpm_R = throttle_right;// + abs(_actuator_out[2])*k1/(1 + throttle_offset) + abs(_actuator_out[3])*k2/(1 + throttle_offset);//get_rotor_rpm(_rpm_R, _actuator_out[2], _actuator_out[3]); // Right rotor motor
    
    _actuator_out[0] = constrain_float(_actuator_out[0], -0.70f, 0.70f);
    _actuator_out[1] = constrain_float(_actuator_out[1], -0.5f, 0.5f);
    _actuator_out[2] = constrain_float(_actuator_out[2], -0.7f, 0.7f);
    _actuator_out[3] = constrain_float(_actuator_out[3], -0.5f, 0.5f);


}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsAirship::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // flap servo 1
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            // flap servo 2
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        case 3:
            // flap servo 3
            rc_write(AP_MOTORS_MOT_3, pwm);
            break;
        case 4:
            // flap servo 4
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        case 5:
            // motor 1 (Left)
            rc_write(AP_MOTORS_MOT_5, pwm);
            break;
        case 6:
            // motor 2 (Right)
            rc_write(AP_MOTORS_MOT_6, pwm);
            break;
        case 7:
            // motor 3 (top)
            rc_write(AP_MOTORS_MOT_7, pwm);
            break;
        default:
            // do nothing
            break;
    }
}

//## Calculate the needed rotor rpm and adjust for the rotor
//  Also includes spinning up and down to reduce wear
float AP_MotorsAirship::get_rotor_rpm(float rpm, float act1, float act2){
/*
    float act_sum = sqrt(act1*act1 + act2*act2);

    // Keep RPM constant as long as the servos stays within a defined radius
    if(act_sum > 0.8){
        rpm += 0.01;
    }
    else if(act_sum < 0.6){
        rpm -= 0.005;
    }
    // Keep it in aloud range.
    rpm = constrain_float(rpm, 0,1);
*/

    // First do test with a constant speed.
    rpm = 0.5;

    return rpm;
}


float AP_MotorsAirship::read_ch(int ch_num)
{
    int16_t forward = hal.rcin->read(ch_num);
	float forward_f = (float(forward) - 982.0f)/1024.0f;
	return forward_f;
}
