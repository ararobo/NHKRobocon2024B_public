/**
 * @file linux_keyboard_driver.hpp
 * @author Gento Aiba (GN10)
 * @brief Linuxのキーボードのイベントを取得するクラス
 * @version 0.1
 * @date 2024-08-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <stdint.h>

class KeyboardDriver
{
private:
    int fd_;

public:
    KeyboardDriver();
    ~KeyboardDriver();

    bool init(const char *device_path);
    bool get_event(uint8_t *key_code, uint8_t *state);
    void release();
};
