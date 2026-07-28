/// @file    AP_MotorsTiltTriY3_Test.cpp
/// @brief   Open-loop test motor class for tilt-tricopter (Y3 layout)

#include "AP_MotorsTiltTriY3_Test.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// ========== 参数表（与正式版相同）==========
const AP_Param::GroupInfo AP_MotorsTiltTriY3_Test::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),

    AP_GROUPINFO("KT1", 1, AP_MotorsTiltTriY3_Test, _k_t1, 6.94e-5f),
    AP_GROUPINFO("KT3", 2, AP_MotorsTiltTriY3_Test, _k_t3, 6.94e-5f),
    AP_GROUPINFO("KD1", 3, AP_MotorsTiltTriY3_Test, _k_d1, 8.47e-7f),
    AP_GROUPINFO("KD3", 4, AP_MotorsTiltTriY3_Test, _k_d3, 8.47e-7f),
    AP_GROUPINFO("L1", 5, AP_MotorsTiltTriY3_Test, _l1, 0.238f),
    AP_GROUPINFO("L2", 6, AP_MotorsTiltTriY3_Test, _l2, 0.058f),
    AP_GROUPINFO("L3", 7, AP_MotorsTiltTriY3_Test, _l3, 0.237f),
    AP_GROUPINFO("MAX_RPS1", 8, AP_MotorsTiltTriY3_Test, _motor_max_rps1, 441.0f),
    AP_GROUPINFO("MAX_RPS3", 9, AP_MotorsTiltTriY3_Test, _motor_max_rps3, 441.0f),
    AP_GROUPINFO("TILT_MAX", 10, AP_MotorsTiltTriY3_Test, _tilt_max_deg, 100.0f),
    AP_GROUPINFO("DEADZN", 11, AP_MotorsTiltTriY3_Test, _deadzone, 1e-3f),
    AP_GROUPINFO("DIR1", 12, AP_MotorsTiltTriY3_Test, _motor_dir1, -1),
    AP_GROUPINFO("DIR2", 13, AP_MotorsTiltTriY3_Test, _motor_dir2, 1),
    AP_GROUPINFO("DIR3", 14, AP_MotorsTiltTriY3_Test, _motor_dir3, 1),
    AP_GROUPINFO("TILT_SW", 15, AP_MotorsTiltTriY3_Test, _rc_sw_ch, 0),
    AP_GROUPINFO("TILT_KNOB", 16, AP_MotorsTiltTriY3_Test, _rc_knob_ch, 0),
    AP_GROUPINFO("HMAIN", 17, AP_MotorsTiltTriY3_Test, _h_main, 0.055f),
    AP_GROUPINFO("HTAIL", 18, AP_MotorsTiltTriY3_Test, _h_tail, 0.063f),

    AP_GROUPEND
};

// ========== Constructor ==========
AP_MotorsTiltTriY3_Test::AP_MotorsTiltTriY3_Test(uint16_t speed_hz) :
    AP_MotorsMulticopter(speed_hz)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// ========== init() ==========
void AP_MotorsTiltTriY3_Test::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    add_motor_num(AP_MOTORS_MOT_1);
    add_motor_num(AP_MOTORS_MOT_2);
    add_motor_num(AP_MOTORS_MOT_3);

    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = true;

    set_update_rate(_speed_hz);

    // 设置舵机行程（centi-degrees）
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorLeft,  _tilt_max_deg.get() * 100);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRight, _tilt_max_deg.get() * 100);
    SRV_Channels::set_angle(SRV_Channel::k_tiltMotorRear,  _tilt_max_deg.get() * 100);

    _mav_type = MAV_TYPE_TRICOPTER;
    set_initialised_ok(frame_class == MOTOR_FRAME_TILTTRI_Y3_TEST);
}

// ========== set_frame_class_and_type() ==========
void AP_MotorsTiltTriY3_Test::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    bool servo_ok = SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorLeft) &&
                    SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRight) &&
                    SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRear);

    set_initialised_ok((frame_class == MOTOR_FRAME_TILTTRI_Y3_TEST) && servo_ok);
}

// ========== set_update_rate() ==========
void AP_MotorsTiltTriY3_Test::set_update_rate(uint16_t speed_hz)
{
    _speed_hz = speed_hz;
    uint32_t mask = (1U << AP_MOTORS_MOT_1) |
                    (1U << AP_MOTORS_MOT_2) |
                    (1U << AP_MOTORS_MOT_3);
    rc_set_freq(mask, _speed_hz);
}

// ========== output_armed_stabilizing()：开环直通 ==========
void AP_MotorsTiltTriY3_Test::output_armed_stabilizing()
{
    float throttle = get_throttle();
    const float compensation_gain = thr_lin.get_compensation_gain();
    throttle *= compensation_gain;

    // ---- 读取 RC 通道 10（索引 9）旋钮，映射为归一化角度指令 ----
    uint16_t knob_pwm = 1500;
    if (hal.rcin->num_channels() > 9) {
        knob_pwm = hal.rcin->read(9);
    }
    float pitch_cmd_norm = (float)(knob_pwm - 1500) / 500.0f;
    pitch_cmd_norm = (pitch_cmd_norm < -1.0f) ? -1.0f : (pitch_cmd_norm > 1.0f ? 1.0f : pitch_cmd_norm);

    float tilt_max_rad = radians(_tilt_max_deg.get());

    // ---- 电机：三个电机完全相等，直接等于油门 ----
    _thrust_1 = throttle;
    _thrust_2 = throttle;
    _thrust_3 = throttle;

    // ---- 舵机：三个舵机联动，直接等于旋钮角度 ----
    float alpha = pitch_cmd_norm * tilt_max_rad;
    alpha = (alpha < -tilt_max_rad) ? -tilt_max_rad : (alpha > tilt_max_rad ? tilt_max_rad : alpha);

    _tilt_angle_front_left  = alpha;
    _tilt_angle_front_right = alpha;
    _tilt_angle_rear        = alpha;
}

// ========== output_to_motors()：统一硬件输出 ==========
void AP_MotorsTiltTriY3_Test::output_to_motors()
{
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            _actuator[AP_MOTORS_MOT_1] = 0.0f;
            _actuator[AP_MOTORS_MOT_2] = 0.0f;
            _actuator[AP_MOTORS_MOT_3] = 0.0f;
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  0);
            break;

        case SpoolState::GROUND_IDLE:
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_1], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_2], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_3], actuator_spin_up_to_ground_idle());
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  0);
            break;

        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_1], thr_lin.thrust_to_actuator(_thrust_1));
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_2], thr_lin.thrust_to_actuator(_thrust_2));
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_3], thr_lin.thrust_to_actuator(_thrust_3));
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  (int16_t)(degrees(_tilt_angle_front_left)  * 100.0f));
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, (int16_t)(degrees(_tilt_angle_front_right) * 100.0f));
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  (int16_t)(degrees(_tilt_angle_rear)        * 100.0f));
            break;
    }

    rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[AP_MOTORS_MOT_1]));
    rc_write(AP_MOTORS_MOT_2, output_to_pwm(_actuator[AP_MOTORS_MOT_2]));
    rc_write(AP_MOTORS_MOT_3, output_to_pwm(_actuator[AP_MOTORS_MOT_3]));
}

// ========== arming_checks() ==========
bool AP_MotorsTiltTriY3_Test::arming_checks(size_t buflen, char *buffer) const
{
    if (!SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorLeft)) {
        hal.util->snprintf(buffer, buflen, "TiltTriY3_Test: no servo for tiltMotorLeft");
        return false;
    }
    if (!SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRight)) {
        hal.util->snprintf(buffer, buflen, "TiltTriY3_Test: no servo for tiltMotorRight");
        return false;
    }
    if (!SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRear)) {
        hal.util->snprintf(buffer, buflen, "TiltTriY3_Test: no servo for tiltMotorRear");
        return false;
    }
    return AP_MotorsMulticopter::arming_checks(buflen, buffer);
}

// ========== _output_test_seq() ==========
void AP_MotorsTiltTriY3_Test::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    switch (motor_seq) {
        case 1: rc_write(AP_MOTORS_MOT_1, pwm); break;
        case 2: rc_write(AP_MOTORS_MOT_2, pwm); break;
        case 3: rc_write(AP_MOTORS_MOT_3, pwm); break;
        default: break;
    }
}

// ========== get_motor_test_order() ==========
uint8_t AP_MotorsTiltTriY3_Test::get_motor_test_order(uint8_t i)
{
    switch (i) {
        case AP_MOTORS_MOT_1: return 1;
        case AP_MOTORS_MOT_2: return 2;
        case AP_MOTORS_MOT_3: return 3;
        default: return 0;
    }
}

// ========== get_motor_mask() ==========
uint32_t AP_MotorsTiltTriY3_Test::get_motor_mask()
{
    uint32_t motor_mask = (1U << AP_MOTORS_MOT_1) |
                          (1U << AP_MOTORS_MOT_2) |
                          (1U << AP_MOTORS_MOT_3);
    uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);
    mask |= AP_MotorsMulticopter::get_motor_mask();
    return mask;
}

// ========== get_roll_factor() ==========
float AP_MotorsTiltTriY3_Test::get_roll_factor(uint8_t i)
{
    switch (i) {
        case AP_MOTORS_MOT_1: return -1.0f;
        case AP_MOTORS_MOT_2: return  1.0f;
        case AP_MOTORS_MOT_3: return  0.0f;
    }
    return 0.0f;
}