/**
 * @file robot2_controller.cpp
 * @author Gento Aiba (GN10)
 * @brief ロボット２のコントロール用プログラム
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "robot2_controller.hpp"
#include "serial_printf.hpp"

Robot2Controller::Robot2Controller()
{
}

void Robot2Controller::control(robot2_wisun_t *robot2_packet)
{
    if (robot2_packet->data.mode == 0) // 手動制御モード
    {
        manual_control(robot2_packet);
    }
    else if (robot2_packet->data.mode == 1) // 自動制御モード
    {
        auto_control(robot2_packet);
    }
}

void Robot2Controller::manual_control(robot2_wisun_t *robot2_packet)
{
    uint8_t action = robot2_packet->data.action & 0b11;
    /* servo */
    for (uint8_t i = 0; i < 4; i++)
    {
        servo_out[i] = servo[action][i];
    }
    can3.sendHexServoTargets_1_4(0, servo_out);
    /* solenoid */
    solenoid_out = solenoid[action];
    can3.sendHexSolenoidTargets(0, solenoid_out);
    /* movement */
    move_robot2(robot2_packet->data.forward, robot2_packet->data.backward,
                robot2_packet->data.left, robot2_packet->data.right, robot2_packet->data.slow);
    /* motor */
    apply_gain_motor();
    can3.sendHexMotorTargets(0, motor_out);
    if (app_configure::mode::debug)
    {
        serial_printf("[DEBUG] R2Controller# motor_out: %d, %d, %d, %d\n", motor_out[0], motor_out[1], motor_out[2], motor_out[3]);
        serial_printf("[DEBUG] R2Controller# servo_out: %d, %d, %d, %d\n", servo_out[0], servo_out[1], servo_out[2], servo_out[3]);
        serial_printf("[DEBUG] R2Controller# solenoid_out: %d\n", solenoid_out);
    }
}

void Robot2Controller::auto_control(robot2_wisun_t *robot2_packet)
{
}

void Robot2Controller::move_robot2(bool forward, bool back, bool left, bool right, bool slow)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        motor[i] = 0.0f; // モーター出力初期化
    }

    if (forward)
    {
        motor[0] += motor_move[uint8_t(slow)];
        motor[1] += motor_move[uint8_t(slow)];
    }
    if (back)
    {
        motor[0] -= motor_move[uint8_t(slow)];
        motor[1] -= motor_move[uint8_t(slow)];
    }
    if (left)
    {
        motor[0] -= motor_rotation[uint8_t(slow)];
        motor[1] += motor_rotation[uint8_t(slow)];
    }
    if (right)
    {
        motor[0] += motor_rotation[uint8_t(slow)];
        motor[1] -= motor_rotation[uint8_t(slow)];
    }
    motor[0] = -motor[0]; // 作動2輪なので左右でモーターの出力を反転する。
}

void Robot2Controller::apply_gain_motor()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        motor_out[i] = int16_t(motor[i] * gain_motor[i]);
    }
}