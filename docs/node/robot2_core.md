# robot2_core
ロボット2の制御をするノード

## Topic通信
[key_to_motion](key_to_motion.md)から制御値を受け取って、各アクチュエーターの出力を導出して送信する。

### publisher
|topic名|型|内容|
|:-:|:-:|:-:|
|robot2/_1/md/target|ros2_htmd_manager_msgs::msg::MdTarget|MDのIDとMDの目標値(制御値or出力値)|
|robot2/_1/servo/angle|main_msgs::msg::ServoAngle|サーボモーターの角度とサーボモーターのチャンネル|
|robot2/_1/solenoid/output|std_msgs/msg/UInt8|ソレノイドバルブの出力(bitごとにシリンダ一つ一つの出力が割り振られる)|

### subscriber
|topic名|型|内容|
|:-:|:-:|:-:|
|robot2/_1/auto_move|std_msgs::msg::UInt8|自動走行の際のゴールのID(1~)(0:停止)|
|robot2/_1/throw|未定boolかboolMultiArray|投擲機構の出力を行う（トリガ）|

## Action通信
https://qiita.com/srs/items/5d7b63bf3a76e8c61d40
これを見て勉強しろ！！！

型：main_msgs::action::Navi

clientです。

### goal
- 対象のロボットID : uint8
- 対象の物体のID : uint8

### result
- ロボットと物体の位置関係 : geometry_msgs::msg::Pose

### feedback
- ロボットと物体の距離 : float64