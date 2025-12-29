#!/bin/bash

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
source install/setup.bash

# ノード起動
timeout 30 ros2 run b64 b64_encode --ros-args -p input_topic:=/test_input &
timeout 30 ros2 run b64 b64_decode --ros-args -p output_topic:=/test_output &
sleep 2

# Test 1: 基本的なエンコード
echo "=== Test 1: Basic encode ==="
ros2 topic pub --once /test_input std_msgs/msg/String "data: 'Hello ROS2'"
sleep 1
timeout 5 ros2 topic echo /b64_encode/output --once > /tmp/b64_test1.log 2>&1
cat /tmp/b64_test1.log | grep 'data:'

# Test 2: 空文字列
echo "=== Test 2: Empty string ==="
ros2 topic pub --once /test_input std_msgs/msg/String "data: ''"
sleep 1
timeout 5 ros2 topic echo /b64_encode/output --once > /tmp/b64_test2.log 2>&1
cat /tmp/b64_test2.log | grep 'data:'

# Test 3: 日本語文字列
echo "=== Test 3: Japanese string ==="
ros2 topic pub --once /test_input std_msgs/msg/String "data: 'こんにちは'"
sleep 1
timeout 5 ros2 topic echo /b64_encode/output --once > /tmp/b64_test3.log 2>&1
cat /tmp/b64_test3.log | grep 'data:'

# Test 4: 長い文字列
echo "=== Test 4: Long string ==="
ros2 topic pub --once /test_input std_msgs/msg/String "data: 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789abcdefghijklmnopqrstuvwxyz'"
sleep 1
timeout 5 ros2 topic echo /b64_encode/output --once > /tmp/b64_test4.log 2>&1
cat /tmp/b64_test4.log | grep 'data:'

# Test 5: 特殊文字
echo "=== Test 5: Special characters ==="
ros2 topic pub --once /test_input std_msgs/msg/String "data: '!@#\$%^&*()_+-=[]{}'"
sleep 1
timeout 5 ros2 topic echo /b64_encode/output --once > /tmp/b64_test5.log 2>&1
cat /tmp/b64_test5.log | grep 'data:'

# Test 6: デコード往復テスト
echo "=== Test 6: Decode roundtrip ==="
timeout 5 ros2 topic echo /test_output --once > /tmp/b64_test6.log 2>&1 &
sleep 1
ros2 topic pub --once /b64_decode/input std_msgs/msg/String "data: 'SGVsbG8gUk9TMg=='"
sleep 2
cat /tmp/b64_test6.log | grep 'data:'

# 画像用ノード起動
timeout 30 ros2 run b64 b64_encode --ros-args -p input_topic:=/image_input -p input_type:=sensor_msgs/msg/Image &
sleep 2

# Test 7: 画像メッセージ (1x1 RGB)
echo "=== Test 7: Image message (1x1 RGB) ==="
ros2 topic pub --once /image_input sensor_msgs/msg/Image "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera'},
  height: 1,
  width: 1,
  encoding: 'rgb8',
  is_bigendian: 0,
  step: 3,
  data: [255, 0, 0]
}"
sleep 1
timeout 5 ros2 topic echo /b64_encode/output --once > /tmp/b64_test7.log 2>&1
cat /tmp/b64_test7.log | grep 'data:'

# Test 8: 画像メッセージ (2x2 RGB)
echo "=== Test 8: Image message (2x2 RGB) ==="
ros2 topic pub --once /image_input sensor_msgs/msg/Image "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera'},
  height: 2,
  width: 2,
  encoding: 'rgb8',
  is_bigendian: 0,
  step: 6,
  data: [255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255]
}"
sleep 1
timeout 5 ros2 topic echo /b64_encode/output --once > /tmp/b64_test8.log 2>&1
cat /tmp/b64_test8.log | grep 'data:'

# Test 9: CompressedImage
echo "=== Test 9: CompressedImage ==="
timeout 30 ros2 run b64 b64_encode --ros-args -p input_topic:=/compressed_input -p input_type:=sensor_msgs/msg/CompressedImage &
sleep 2
ros2 topic pub --once /compressed_input sensor_msgs/msg/CompressedImage "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera'},
  format: 'jpeg',
  data: [255, 216, 255, 224, 0, 16]
}"
sleep 1
timeout 5 ros2 topic echo /b64_encode/output --once > /tmp/b64_test9.log 2>&1
cat /tmp/b64_test9.log | grep 'data:'

echo "=== All tests completed ==="
