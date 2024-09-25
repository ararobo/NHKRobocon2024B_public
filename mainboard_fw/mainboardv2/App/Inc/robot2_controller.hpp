/**
 * @file robot2_controller.hpp
 * @author Gento Aiba (GN10)
 * @brief ロボット２のコントロール用プログラム
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <stdint.h>
#include "app_configure.hpp"
#include "wisun_data_configure.hpp"
#include "can3_manager.hpp"

extern CAN3Manager can3;

class Robot2Controller
{
private:
    float gain_motor[4] = {2000.0f, 2000.0f, 2000.0f, 2000.0f}; // モーターゲイン[duty]{右輪, 左輪, 未定, 未定}
    float motor_rotation[2] = {1.0f, 0.7f};                     // 回転ゲイン[slow]
    float motor_move[2] = {1.0f, 0.7f};                         // 移動ゲイン[slow]
    float motor[4] = {0.0f, 0.0f, 0.0f, 0.0f};                  // モーター出力[duty]
    /* 各サーボモーターのPWMのON時間[us]（モードごと） */
    uint16_t servo[4][4] = {{330, 330, 135, 540}, {200, 200, 135, 540}, {330, 330, 307, 372}, {200, 200, 307, 372}}; // [action][servo] //
    /* 各ソレノイドのモードごとの出力 */
    uint8_t solenoid[4] = {0, 0, 0, 0}; // [action]

    /* 出力値 */
    uint16_t servo_out[4]; // サーボ出力
    int16_t motor_out[4];  // モーター出力
    uint8_t solenoid_out;  // ソレノイド出力

    /**
     * @brief 自動制御
     *
     * @param robot2_packet ロボット2パケット
     */
    void auto_control(robot2_wisun_t *robot2_packet);

    /**
     * @brief モーターゲイン適用して、motor_outに出力
     */
    void apply_gain_motor();

    /**
     * @brief 移動する
     *
     */
    void move_robot2(bool forward, bool back, bool left, bool right, bool slow);

public:
    Robot2Controller();
    void set_id(uint8_t id);
    void control(robot2_wisun_t *robot2_packet);
    /**
     * @brief 手動制御
     *
     * @param robot2_packet ロボット2パケット
     */
    void manual_control(robot2_wisun_t *robot2_packet);
};
