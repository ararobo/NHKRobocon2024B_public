# robot1_navi

# Topic通信
距離センサx4の値・ジャイロセンサの値・キーボードの状態を用いてロボット１の移動ベクトルと投擲命令を生成し、X,Y,回転,投擲の出力値を[robot1_core](robot1_core.md)送信する

# subscriber
|topic名|型|内容|
|:-:|:-:|:-:|
|robot1/distance|std_msgs::msg::Float32MultiArray|ロボット1の距離センサの値x4[front, right, back, left]|
|robot1/jyro|std_msgs::msg::Float32|ロボットの向き[rad]|
|key_state|main_msgs::msg::KeyState|いくつかのキーの状態[bool array]|

# publisher
|topic名|型|内容|
|:-:|:-:|:-:|
|robot1/movement|geometry_msgs::msg::Pose2D|ロボットの移動ベクトル[x,y,θ]|
|robot1/throw_1|std_msgs::msg::Bool|投擲機構の出力を行う（トリガ）|
|robot1/throw_2|std_msgs::msg::Bool|投擲機構の出力を行う（トリガ）|