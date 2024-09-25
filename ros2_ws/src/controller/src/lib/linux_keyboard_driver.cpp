/**
 * @file linux_keyboard_driver.cpp
 * @author Gento Aiba (GN10)
 * @brief Linuxのキーボードのイベントを取得するクラス
 * @version 0.1
 * @date 2024-08-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <fcntl.h>
#include <linux/input.h>
#include <unistd.h>
#include "lib/linux_keyboard_driver.hpp"

KeyboardDriver::KeyboardDriver()
{
}

KeyboardDriver::~KeyboardDriver()
{
    release();
}

bool KeyboardDriver::init(const char *device_path)
{
    fd_ = open(device_path, O_RDONLY);
    if (fd_ < 0)
    {
        return false;
    }
    // 非ブロックモード
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    return true;
}

bool KeyboardDriver::get_event(uint8_t *key_code, uint8_t *key_state)
{
    struct input_event ev_;

    ssize_t n = read(fd_, &ev_, sizeof(ev_));

    if (n == (ssize_t)-1)
    {
        return false;
    }
    else if (n != sizeof(ev_))
    {
        return false;
    }

    if (ev_.type == EV_KEY)
    {
        if (0 < ev_.code && ev_.code < 126)
        {
            *key_code = ev_.code;
            *key_state = ev_.value;
            return true;
        }
    }
    return false;
}

void KeyboardDriver::release()
{
    close(fd_);
}