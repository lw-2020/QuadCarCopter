#include "AC_QuadCarControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

#include "stdio.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AC_QuadCarControl::var_info[] = {

    AP_SUBGROUPINFO(_pid_speed, "SPD_", 1, AC_QuadCarControl, AC_PID),

    AP_SUBGROUPINFO(_pid_turn, "TRN_", 2, AC_QuadCarControl, AC_PID),

    AP_GROUPINFO("MAX_SPEED", 3, AC_QuadCarControl, _max_speed, AC_BALANCE_MAX_SPEED),

    AP_GROUPINFO("TAR_SP_X", 4, AC_QuadCarControl, Target_Velocity_X, AC_BALANCE_TARGET_X_SPEED),

    AP_GROUPINFO("TAR_SP_Z", 5, AC_QuadCarControl, Target_Velocity_Z, AC_BALANCE_TARGET_Z_SPEED),

    AP_GROUPINFO("ACCEL_MAX", 6, AC_QuadCarControl, _max_accel, AC_BALANCE_MAX_ACCEL),

    AP_GROUPEND
};

//xiugai


AC_QuadCarControl::AC_QuadCarControl(AP_Motors* motors, AP_AHRS_View* ahrs)
    : _pid_angle(AC_BALANCE_ANGLE_P, 0, AC_BALANCE_ANGLE_D, 0, 0, 0, 0, 0)
    , _pid_speed(AC_BALANCE_SPEED_P, AC_BALANCE_SPEED_I, 0, AC_BALANCE_SPEED_IMAX, 0, 0, 0, 0)
    , _pid_turn(AC_BALANCE_TURN_P, 0, AC_BALANCE_TURN_D, 0, 0, 0, 0, 0)
    , _pid_roll(AC_BALANCE_ROLL_P, AC_BALANCE_ROLL_I, AC_BALANCE_ROLL_D, 0, AC_BALANCE_ROLL_IMAX, 0, 0, 0)
    , _motors(motors)
    , _ahrs(ahrs)
{
    AP_Param::setup_object_defaults(this, var_info);

    _dt = 1.0f / 200.0f;

    speed_low_pass_filter.set_cutoff_frequency(30.0f);
    speed_low_pass_filter.reset(0);

    _moveflag_x = moveFlag::none;
    _moveflag_z = moveFlag::none;
    
    _last_target_speed = 0.0f;

    balanceMode = BalanceMode::ground;

    stop_quadcar_control = false;

    force_stop_quadcar_control = false;
}

void AC_QuadCarControl::init()
{
    quadcarCAN = AP_QuadCarCAN::get_singleton();
}

// 平滑目标速度函数，使加速度限制在_max_accel范围内
float AC_QuadCarControl::smooth_target_speed(float target_speed)
{
    float smoothed_speed = _last_target_speed;
    float accel_limit = _max_accel * _dt;
    
    // 根据加速度限制计算平滑后的速度
    if (target_speed > _last_target_speed) {
        smoothed_speed = _last_target_speed + MIN(target_speed - _last_target_speed, accel_limit);
    } else if (target_speed < _last_target_speed) {
        smoothed_speed = _last_target_speed - MIN(_last_target_speed - target_speed, accel_limit);
    }
    
    _last_target_speed = smoothed_speed;
    return smoothed_speed;
}

/**************************************************************************
Function: Speed PI control
Input   : encoder_left：Left wheel encoder reading；encoder_right：Right wheel encoder reading
Output  : Speed control PWM
函数功能：速度控制PWM
入口参数：encoder_left：左轮编码器读数；encoder_right：右轮编码器读数
返回  值：速度控制PWM
**************************************************************************/
float AC_QuadCarControl::Velocity(float encoder_left, float encoder_right)
{
    float velocity_out;
    float Encoder_error;
    float target_speed = 0.0f;
    float encoder_movement;

    //================遥控前进后退部分====================//
    encoder_movement = (float)_movement_x / 500.0f * Target_MAX_Velocity_X;

    //================速度PI控制器=====================//
    
    // 根据移动标志确定目标速度
    switch (_moveflag_x) {
        case moveFlag::moveFront:
            target_speed = Target_Velocity_X;
            break;
        case moveFlag::moveBack:
            target_speed = -Target_Velocity_X;
            break;
        default:
            target_speed = 0.0f;
            break;
    }
    
    // 平滑目标速度，限制加速度
    float smoothed_target = smooth_target_speed(target_speed);

    // 获取最新速度偏差=目标速度-测量速度（左右编码器之和）
    Encoder_error = encoder_movement - (encoder_left + encoder_right);

    float Encoder_filter = speed_low_pass_filter.apply(Encoder_error, _dt);

    // 使用平滑后的目标速度
    velocity_out = _pid_speed.update_all(smoothed_target, Encoder_filter, _dt);

    if (stop_quadcar_control || Flag_Stop || force_stop_quadcar_control) {
        _pid_speed.reset_I();
        Encoder_error = 0;
        Encoder_filter = 0;
        encoder_movement = 0;
        velocity_out = 0;
    }

    return velocity_out;
}

/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
函数功能：转向控制
入口参数：Z轴陀螺仪
返回  值：转向控制PWM
**************************************************************************/
float AC_QuadCarControl::Turn(float gyro)
{
    float turn_target;
    float turn_out;
    
    //===================遥控左右旋转部分=================//
    turn_target = (float)_movement_z / 500.0f * Target_MAX_Velocity_Z;

    //===================转向PD控制器=================//
    turn_out = turn_target * _pid_turn.kP() + gyro * _pid_turn.kD(); // 结合Z轴陀螺仪进行PD控制

    if (stop_quadcar_control || Flag_Stop || force_stop_quadcar_control) {
        turn_out = 0;
        turn_target = 0;
    }

    return turn_out;
}

void AC_QuadCarControl::update(void)
{
    if (_motors == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "_motors = nullptr");
        return;
    }

    if (quadcarCAN == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "quadcarCAN = nullptr");
        return;
    }
    
    if (_ahrs == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "_ahrs = nullptr");
        return;
    }

    static float gyro_z;
    static float wheel_left_f, wheel_right_f;
    static float motor_target_left_f, motor_target_right_f;
    const float  max_scale_value = 10000.0f;

    gyro_z = _ahrs->get_gyro_latest()[2];

    // 转速缩小1000倍
    wheel_left_f  = (float)quadcarCAN->getSpeed(1) / max_scale_value;
    wheel_right_f = -(float)quadcarCAN->getSpeed(2) / max_scale_value;

    // 速度环PID控制
    control_velocity = Velocity(wheel_left_f, wheel_right_f);

    // 转向环PID控制
    control_turn = Turn(gyro_z);

    // motor值正数使小车前进，负数使小车后退, 范围【-1，1】
    motor_target_left_f  = control_velocity + control_turn; // 计算左轮电机最终PWM
    motor_target_right_f = control_velocity - control_turn; // 计算右轮电机最终PWM

    int16_t motor_target_left_int  = (int16_t)(motor_target_left_f * max_scale_value);
    int16_t motor_target_right_int = -(int16_t)(motor_target_right_f * max_scale_value);

    // 限制电机输入范围
    motor_target_left_int = constrain_int16(motor_target_left_int, -_max_speed, _max_speed);
    motor_target_right_int = constrain_int16(motor_target_right_int, -_max_speed, _max_speed);

    // 最终的电机输入量
    quadcarCAN->setCurrent(1, (int16_t)motor_target_left_int);
    quadcarCAN->setCurrent(2, (int16_t)motor_target_right_int);

    pilot_control();

    debug_info();
    
    // 读取遥控器的CH7通道，判断是否停止小车控制
    // 这里的1700是一个经验值，可以根据实际情况进行调整
    // 例如，如果遥控器的CH8通道值大于1700，则停止小车控制
    // 如果遥控器的CH8通道值小于1700，则恢复小车控制
    if (hal.rcin->read(CH_8) > 1700) {
        stop_quadcar_control = true;
    } else {
        stop_quadcar_control = false;
    }

    // 新增：通过CH_7控制电动泵PWM输出
    // 假设PWM输出通道为5（实际请根据硬件分配）
    const uint8_t PUMP_PWM_CHANNEL = 7;
    if (hal.rcin->read(CH_9) > 1700) {
        // 打开电动泵，输出高占空比PWM
        SRV_Channels::set_output_pwm_chan(PUMP_PWM_CHANNEL, 2000); // 2000us脉宽
    } else {
        // 关闭电动泵，输出低占空比PWM
        SRV_Channels::set_output_pwm_chan(PUMP_PWM_CHANNEL, 1000); // 1000us脉宽
    }

}

void AC_QuadCarControl::pilot_control()
{
    int16_t pwm_x = hal.rcin->read(CH_2) - 1500;
    int16_t pwm_z = hal.rcin->read(CH_4) - 1500;
    int16_t pwm_y = hal.rcin->read(CH_1) - 1500;

    if (pwm_x < 50 && pwm_x > -50) {
        _movement_x = 0;
    }
    else if (abs(pwm_x) > 500) {
        _movement_x = 0;
    }
    else {
        _movement_x = pwm_x;
    }

    if (pwm_z < 50 && pwm_z > -50) {
        _movement_z = 0;
    }
    else if (abs(pwm_z) > 500) {
        _movement_z = 0;
    }
    else {
        _movement_z = pwm_z;
    }

    if (pwm_y < 20 && pwm_y > -20) {
        _movement_y = 0;
    }
    else if (abs(pwm_y) > 500) {
        _movement_y = 0;
    }
    else {
        _movement_y = pwm_y;
    }

}

void AC_QuadCarControl::debug_info()
{
    //调试用
    static uint16_t cnt = 0;
    cnt++;
    if (cnt > 200) {
        cnt = 0;
        gcs().send_text(MAV_SEVERITY_NOTICE, "left_real_speed=%d", quadcarCAN->getSpeed(1));
        gcs().send_text(MAV_SEVERITY_NOTICE, "right_real_speed=%d", quadcarCAN->getSpeed(2));
        gcs().send_text(MAV_SEVERITY_NOTICE, "target_speed_x=%.2f", (float)Target_Velocity_X);
        gcs().send_text(MAV_SEVERITY_NOTICE, "move_flag_x=%d, move_flag_z=%d", _moveflag_x, _moveflag_z);
    }

}
