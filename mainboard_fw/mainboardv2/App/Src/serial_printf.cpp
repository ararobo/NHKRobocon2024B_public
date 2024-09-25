/**
 * @file serial_printf.cpp
 * @author Gento Aiba (GN10)
 * @brief UARTを用いたprintf関数
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "serial_printf.hpp"

template void serial_printf<>(const std::string &fmt);
template void serial_printf<unsigned char, unsigned char, unsigned char, unsigned char, unsigned char, unsigned char>(const std::string &fmt, unsigned char, unsigned char, unsigned char, unsigned char, unsigned char, unsigned char);
template void serial_printf<unsigned char, unsigned char, unsigned char, unsigned char>(const std::string &fmt, unsigned char, unsigned char, unsigned char, unsigned char);