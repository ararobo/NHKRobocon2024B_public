# keyboard_driver
キーボードの状態をtopic通信でpublishします

## 機能
- キーボードの状態をROS2のTopic通信で送信する

## ノード
- keyboard_driver
キーボードの状態をTopic通信かwi-sunでpublishします。

## Topic通信

### publisher
|topic名|型|内容|
|:-:|:-:|:-:|
|key_state|main_msgs::msg::KeyState|いくつかのキーの状態[bool array]|

## wi-sun
シリアル通信を使ってやり取りをします。
UDPとして以下のように送信します。

|port|packet_name|description|structure|dlc|
|:-:|:-:|:-:|:-:|
|21410|key_state|キーボードの状態|[キーコード:uint8, キーステート:uint8]|2|