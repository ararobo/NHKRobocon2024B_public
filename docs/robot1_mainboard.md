# robot1 mainboard

|受信|送信|説明|送信データフレーム|周期[ms]|
|:-:|:-:|:-:|:-:|:-:|
|UDP|CAN|ポートが一致すればパケットをCANのデータフレームにセットして送信|Nomal CAN packet|5|
|CAN|UDP|CANIDが一致すればパケットをUDPのデータフレームにセットして送信|header, DLC, {CANid, DLC, data, END} ...|1|
|920|UDP|コントローラーの状態を受信したら、そのまま送信|header, DLC, data|1|
|UDP|920|UDPのデータをそのまま920で送信|header, DLC, data|1|