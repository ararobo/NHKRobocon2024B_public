# robot1_core
ロボット１の制御をするノード

## Topic通信
[robot1_navi](robot1_navi.md)から制御値を受け取って、各アクチュエーターの出力を導出して送信する。

### subscriber
|topic名|型|内容|
|:-:|:-:|:-:|
|robot1/movement|geometry_msgs::msg::Pose2D|ロボットの移動ベクトル[x,y,θ]|
|robot1/throw_1|std_msgs::msg::Bool|投擲機構の出力を行う（トリガ）|
|robot1/throw_2|std_msgs::msg::Bool|投擲機構の出力を行う（トリガ）|

### publisher
|topic名|型|内容|
|:-:|:-:|:-:|
|robot1/md/target|ros2_htmd_manager_msgs::msg::MdTarget|MDのIDとMDの目標値(制御値or出力値)|
|robot1/servo/angle|main_msgs::msg::ServoAngle|サーボモーターの角度とサーボモーターのチャンネル|
|robot1/solenoid/output|std_msgs/msg/UInt8|ソレノイドバルブの出力(bitごとにシリンダ一つ一つの出力が割り振られる)|

## 処理
### ロボットの移動[robot1/movement]
送られてきたx,y,θから各オムニホイールx4の出力を計算して、robot1/md/targetにpublishする。
### 投擲[robot1/throw]
1. シリンダを駆動
2. リミットスイッチ１が当たるまでモーターを逆回転
3. シリンダを駆動
4. リミットスイッチ２が当たるまでモーターを正回転