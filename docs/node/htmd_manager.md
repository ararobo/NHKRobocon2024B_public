# HTMD Manager Node
モータードライバーの管理を担うノードです。
送信と受信でノードが別れていますが、ここで両方について説明します。

## parameter(sender)
xはMDのID
|parameter名|型|説明|
|:-:|:-:|:-:|
|enable.md_x_init|bool|ノード起動時に初期化するMDを設定する|
|default.md_x_pid.Kp|double|モーターのPID制御の比例ゲイン|
|default.md_x_pid.Ki|double|モーターのPID制御の積分ゲイン|
|default.md_x_pid.Kd|double|モーターのPID制御の微分ゲイン|
|default.md_x_mode.incremental_encoder|bool|MDがインクリメンタルエンコーダーの値をCAN通信で送信するかどうか|
|default.md_x_mode.absolute_encoder|bool|MDがアブソルートエンコーダーの値をCAN通信で送信するかどうか|
|default.md_x_mode.reverse_encoder|bool|エンコーダーの値を反転するかどうか|
|default.md_x_mode.brake|bool|モーター停止時にブレーキをかけるかどうか|
|default.md_x_mode.pid|bool|PID制御を行うかどうか|
|default.md_x_mode.current|bool|電流値をCAN通信で送信するかどうか|
|default.md_x_mode.torque_control|bool|トルク制御を行うかどうか|
|default.md_x_mode.state|bool|MD状態を定期的にCAN通信で送信するかどうか|
|default.md_x_mode.max_acceleration|int|モーターの最大加速(制御周期あたりのduty比の変化量)|
|default.md_x_mode.max_current|double|モーターの消費電流の最大値|
|default.md_x_mode.report_rate|int|センサーの状態を何ミリ秒おきに送信するか|
|default.md_x_mode.max_output|int|最大出力(duty比)
|default.md_x_mode.motor_transfer_coefficient|double|モーターの制御値とエンコーダの値の相関関数|

## parameter(receiver)
|parameter名|型|説明|
|:-:|:-:|:-:|
|md_num|int|MDの数を指定してMDから受信した情報をTopicで送信する際に使用するTopicを作成します|
|motor_transfer_coefficient_quality|double|モーターの伝達係数の通信で小数を送信する際に何倍にして送信するか|

## Topic通信

MDの制御や、管理、監視をROS2のTopic通信を通して行うことができます。

- 制御

|topic名|型|内容|
|:-:|:-:|:-:|
|robot1/md/target|ros2_htmd_manager_msgs::msg::MdTarget|MDのIDとMDの目標値(制御値or出力値)|

- 管理

|topic名|型|内容|
|:-:|:-:|:-:|
|robot1/md/pid_gain|ros2_htmd_manager_msgs::msg::PidGain|MDのIDとPID制御のゲイン|
|robot1/md/mode|ros2_htmd_manager_msgs::msg::MdMode|MDのIDとMDのモード|

- 監視

n : MDのID

|topic名|型|内容|
|:-:|:-:|:-:|
|robot1/md/_n/state|ros2_htmd_manager_msgs::msg::MdState|MDの状態(状態コードと温度)|
|robot1/md/_n/sensor|ros2_htmd_manager_msgs::msg::MdSensor|MDのセンサーの状態(リミットスイッチ, エンコーダー, 電流値)|