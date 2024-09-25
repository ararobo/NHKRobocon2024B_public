/**
 * @file wisun_tx.hpp
 * @author Gento Aiba (GN10)
 * @brief Wi-SUNモジュールにコマンドを送信するプログラム
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "stdio.h"
#include "stdint.h"
#include <stdlib.h>
#include <signal.h>
#define DEBUG_PACKET_SIZE 255
#define DEBUG_PACKET_META_DATA_SIZE 2

uint8_t data_kind_buffer;
volatile sig_atomic_t e_flag;
uint8_t debug_packet_buffer_tx[DEBUG_PACKET_SIZE + DEBUG_PACKET_META_DATA_SIZE];

/**
 * @brief デバッグパケットを構築する
 *
 * @param data_kind 通信データの種類[wisun_debug...etc.]
 * @param data データ
 * @param data_size データ長[byte]
 * @return true 問題なし
 * @return false 問題あり
 */
bool encode_debug_packet(uint8_t data_kind, uint8_t *data, uint8_t data_size);

void abrt_handler(int sig);
