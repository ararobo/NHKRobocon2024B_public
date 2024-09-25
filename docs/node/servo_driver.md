# Servo Driver Node
サーボーモーターを回すノード
Topic通信でサーボモーターの角度を受信して、その角度にサーボモーターを回す。
基盤への通信は２種類用いることができる。
1. CAN通信
2. UDP通信（Wifi）

## Topic通信
subscriber
|topic名|型|内容|
|:-:|:-:|:-:|
|servo/angle|main_msgs::msg::ServoAngle|サーボモーターの角度とサーボモーターのチャンネル|

## CAN通信
x: サーボモータのID
|can id|dlc|packet name|description|structure|
|:-:|:-:|:-:|:-:|:-:|
|0x11x|2|servo_angle|サーボモーターの角度|[uint16_t]|

## UDP通信
|port|id|dlc|packet_name|description|structure|
|:-:|:-:|:-:|:-:|:-:|:-:|
|21400|0x11x|2|servo_angle|サーボモーターの角度|[uint16_t]|