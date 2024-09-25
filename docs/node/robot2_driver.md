# robot2_driver
ロボット２の制御値をまとめてメイン基板に送信するノード
## Topic通信
subscriberのみあります。
|topic名|型|内容|
|:-:|:-:|:-:|
|robot2/control|main_msgs::msg::robot2_control|サーボの状態2bit,方向2bit|