# aruco_detector
arucoマーカーを認識して座標を取得するノード

## Topic通信
ロボット２の位置を送信します。

### publisher
|topic名|型|内容|
|:-:|:-:|:-:|
|robot2/_1/aruco|main_msgs::msg::ArucoMarkers|[マーカーの位置[x,y,z],マーカーの向き[x,y,z,w],マーカーのID]|

## 処理
1. OpenCVのこの[Arucoマーカの認識について](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)の記事を参考に簡単に認識し、深度画像を用いて、三次元座標を正確に導出する。
2. 導出した三次元座標をpublishする。