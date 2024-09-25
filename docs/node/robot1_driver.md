# robot1_driver
ロボット１のサーボモーターとLEDとソレノイドの制御値をメイン基板に送信するノード
## Topic通信
subscriberのみあります。
|topic名|型|内容|
|:-:|:-:|:-:|
|robot1/solenoid/output|std_msgs/msg/u_int8|ソレノイドバルブの出力(bitごとにシリンダ一つ一つの出力が割り振られる)|
|robot1/servo|main_msgs::msg::ServoAngle|サーボモーターの角度とサーボモーターのチャンネル|