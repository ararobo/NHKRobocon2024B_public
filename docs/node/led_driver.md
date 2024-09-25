# LED driver
フルカラーLEDのモードをCAN又はUDPで送信しLEDの状態を変化させます。

## Topic通信

### subscriber
|topic名|型|内容|
|:-:|:-:|:-:|
|robot1/led/mode|std_msgs::msg::UInt8|LEDのモード|

## CAN通信
|can id|dlc|packet name|description|structure|
|:-:|:-:|:-:|:-:|:-:|
|0x600|1|LED_mode|LEDのモード|uint8|

## UDP通信
|port|id|dlc|packet_name|description|structure|
|:-:|:-:|:-:|:-:|:-:|:-:|
|21400|0x600|1|LED_mode|LEDのモード|uint8|

## modeについて
|mode|LEDの状態|
|:-:|:-:|
|0|消灯|
|1|赤|
|2|青|
|3|ゲーミング|
|4|あたおか|