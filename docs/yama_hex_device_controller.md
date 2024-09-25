# Yama Hex Device Controller
サーボモーターと電磁弁とモータを扱う基盤。
DCモーターx4
サーボモータx3
電磁弁x4

## CAN通信
|can id|dlc|packet name|description|structure|
|:-:|:-:|:-:|:-:|:-:|
|0x13x|8|robot2_control|ロボット２の制御値|[モーターx4:int8x4, サーボx4:uint8x3, ソレノイドx8:uint8]|
|0x200|2|manage_device_init_command|デバイスの初期化|device[uint8], command[uint8]|

## 処理
1. 初期化コマンド
2. 送られてきた制御値をモータ・サーボ・ソレノイドに分類
3. それぞれの制御値を出力
