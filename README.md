# b64

[![Test](https://github.com/NOPLAB/mypkg/actions/workflows/test.yml/badge.svg)](https://github.com/NOPLAB/mypkg/actions/workflows/test.yml)

任意のトピックをbase64エンコード/デコードするROS2パッケージ

## ノード

### b64_encode

任意のトピックをsubscribeし、base64エンコードしてStringとしてpublish

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `input_topic` | `/input` | 入力トピック名 |
| `input_type` | `std_msgs/msg/String` | 入力メッセージ型 |

| トピック | 型 | 説明 |
|---------|-----|------|
| `~/output` | `std_msgs/msg/String` | base64エンコード済み文字列 |

### b64_decode

base64文字列をsubscribeし、デコードして任意の型としてpublish

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `output_topic` | `/output` | 出力トピック名 |
| `output_type` | `std_msgs/msg/String` | 出力メッセージ型 |

| トピック | 型 | 説明 |
|---------|-----|------|
| `~/input` | `std_msgs/msg/String` | base64エンコード済み文字列 |

## Launch

### b64.launch.py

encode/decodeノードを同時に起動

| 引数 | デフォルト | 説明 |
|------|-----------|------|
| `input_topic` | `/input` | エンコーダーの入力トピック |
| `input_type` | `std_msgs/msg/String` | 入力メッセージ型 |
| `output_topic` | `/output` | デコーダーの出力トピック |
| `output_type` | `std_msgs/msg/String` | 出力メッセージ型 |

```bash
ros2 launch b64 b64.launch.py
```

## 使用例

### 文字列のエンコード

`/chatter`トピックの文字列をbase64エンコードして`~/output`にpublish：

```bash
ros2 run b64 b64_encode --ros-args -p input_topic:=/chatter
```

### 画像のエンコード

カメラ画像をbase64エンコード。`input_type`で入力メッセージ型を指定：

```bash
ros2 run b64 b64_encode --ros-args \
  -p input_topic:=/camera/image_raw \
  -p input_type:=sensor_msgs/msg/Image
```

### デコードして復元

base64文字列を画像メッセージにデコードして`/camera/image_restored`にpublish：

```bash
ros2 run b64 b64_decode --ros-args \
  -p output_topic:=/camera/image_restored \
  -p output_type:=sensor_msgs/msg/Image
```

### Launchファイルで画像をエンコード/デコード

エンコーダーとデコーダーを同時に起動し、画像トピックをbase64経由で中継する例：

```bash
ros2 launch b64 b64.launch.py \
  input_topic:=/camera/image_raw \
  output_topic:=/camera/image_restored \
  input_type:=sensor_msgs/msg/Image \
  output_type:=sensor_msgs/msg/Image
```

`/camera/image_raw` → base64エンコード → base64デコード → `/camera/image_restored` の流れでデータが中継される。

### テスト環境

- ROS2 Humble (`ros:humble`)
- ROS2 Jazzy (`ros:jazzy`)

## ライセンスおよびコピーライト

© 2025 nop

このプロジェクトはMITライセンスの下で公開されています。詳細は[LICENSE](LICENSE)ファイルをご覧ください。
