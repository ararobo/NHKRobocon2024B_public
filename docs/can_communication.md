# CAN通信
2024年のCAN通信の定義です。
制御はすべてPC(メイン基板)で行う。

## CAN-ID
11bitの標準IDを定義します。

|id|10(MSB)|9|8|7|6|5|4|3|2|1|0(LSB)|
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
|用途|1.direction|2.dev_kind|2.|2.|3.dev_id|3.|3.|3.|4.detail|4.|4.|

### 1. direction
制御基板(Master)とその他基板(Slave)の通信方向を0と1で示す。

|向き|1.の値|
|:-:|:-:|
|Master->Slave(PCからMD等)|0|
|Slave->Master(MDからPC等)|1|

### 2. dev_kind
デバイスの種類を0~7表す。

|デバイス|2.の値|
|:-:|:-:|
|共通|0|
|モータードライバー|1|
|サーボモータードライバー|2|
|ソレノイドドライバー|3|
|LEDドライバー|4|

### 3. dev_id
デバイスのIDを0~Fで示す。

### 4. detail
通信内容を0~7で表す。
それぞれのデバイスに対して定義される。

## モータードライバー
多くの通信が定義されていますが、MDによって有効なものと無効なものがあります。

|4.の値|データ長|内容|データフレーム|
|:-:|:-:|:-:|:-:|
|0|2|モーターの出力(md_id:0~F)|出力値 : int16|
|0|8|4つのモーターの出力(md_id:0\~3, ..., 3C\~3F)|出力値x4 : int16[4]|
|1|1|初期化|0(固定) : uint8|
|2|8|MDのモード|mode_code : uint8[8]|
|3|6|Pゲイン|P_gain : float|
|4|6|Iゲイン|I_gain : float|
|5|6|Dゲイン|D_gain : float|
|6|1|リミットスイッチ|limSW1(0x1)とlimSW2(0x2)の論理和 : uint8|
|6|3|リミットスイッチとエンコーダー|limSW1(0x1)とlimSW2(0x2)の論理和 : uint8, encoder : int16|
|6|7|リミットスイッチとエンコーダーと電流値|limSW1(0x1)とlimSW2(0x2)の論理和 : uint8, encoder : int16, current : uint16|
|7|1|MDの状態|状態 : uint8|
|7|2|MDの状態と温度|状態 : uint8, 温度 : uint16|

#### MDのモードについて
uint8[8]とされているが、中身がわからないと思うので、変換する際に使用するunionを書いておく
```c++
struct md_mode_flags
{
    unsigned char incremental_encoder : 1;
    unsigned char absolute_encoder : 1;
    unsigned char reverse_encoder : 1;
    unsigned char brake : 1;
    unsigned char pid : 1;
    unsigned char stop_by_limit_switch : 1; // 0が送信されるまで出力を無効にする。
    unsigned char torque_control : 1;
    unsigned char state : 1;
};

union md_mode_t
{
    struct
    {
        struct md_mode_flags flags;
        struct
        {
            uint8_t max_acceleration;
            uint8_t max_current;
            uint8_t report_rate;
            uint16_t max_output;
            uint16_t motor_transfer_coefficient;
        } values;
    };
    uint8_t code[8];
};
```

使用方法
```c++
md_mode_t md_mode;
md_mode.flags.incremental_encoder = 1; // インクリメンタルエンコーダーを有効にする。
/* その他フラグに値を代入する */
md_mode.values.max_acceleration = 255; // 台形制御の最大加速を255に設定。
/* その他パラメーターに値を代入する */
// ESP32の例
CAN.beginPacket(0x82); // md_id=0としてIDを指定
for (uint8 i = 0; i < 8; i++) // 8byte
{
	CAN.write(md_mode.code[i]); // unionで変換されたデータを送信する
}
CAN.endPacket();
```

## サーボモータードライバー

|4.の値|データ長|内容|データフレーム|
|:-:|:-:|:-:|:-:|
|0|2|パルス幅(servo_id:0~F)[us]|パルス幅 : uint16|
|0|8|パルス幅(servo_id:0\~3, ..., 3C\~3F)[us]|パルス幅x4 : uint16[4]|
|1|1|初期化|0(固定) : uint8|
|2|6|周波数|freq : float|

## ソレノイドドライバー

|4.の値|データ長|内容|データフレーム|
|:-:|:-:|:-:|:-:|
|0|1|出力x8|各bitごとに一つのソレノイドに対応する : uint8|
|1|1|初期化|0(固定) : uint8|

## LEDドライバー

|4.の値|データ長|内容|データフレーム|
|:-:|:-:|:-:|:-:|
|0|1|点灯モード|モード(256パターン) : uint8|
|1|1|初期化|0(固定) : uint8|