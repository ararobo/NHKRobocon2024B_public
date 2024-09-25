# ロボット２のメイン基板
ロボット２の無線通信、LED制御を行います。

基板データ:

## 処理内容
1. ロボット１からwi-sunで制御信号を受信する。
2. 受信した制御信号をCAN通信で送信する。
3. LED制御をする。

|受信|送信|説明|送信データフレーム|周期[ms]|
|:-:|:-:|:-:|:-:|:-:|
|UDP|CAN|ポートが一致すればパケットをCANのデータフレームにセットして送信|Nomal CAN packet|5|
|CAN|UDP|CANIDが一致すればパケットをUDPのデータフレームにセットして送信|header, DLC, {CANid, DLC, data, END} ...|1|

## wi-sun
### 通信内容
- モーターの出力
- サーボモーターの角度
- シリンダの出力
- LEDのモード

## CAN通信
|can id|dlc|packet name|description|structure|
|:-:|:-:|:-:|:-:|:-:|
|0x13x|8|robot2_control|ロボット２の制御値|[モーターx4:int8x4, サーボx4:uint8x3, ソレノイドx8:uint8]|
|0x200|2|manage_device_init_command|デバイスの初期化|device[uint8], command[uint8]|

## UDP通信
|port|packet_name|description|structure|dlc|
|:-:|:-:|:-:|:-:|:-:|
|21420|robot2_control|ロボット２の制御値|[モーターx4:int8x4, サーボx4:uint8x3, ソレノイドx8:uint8]|8|