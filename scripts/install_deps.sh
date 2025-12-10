#!/bin/bash

# 安装依赖脚本 (Ubuntu 20.04)

set -e

echo "========================================="
echo "  3D 自主探索系统 - 依赖安装"
echo "  目标系统: Ubuntu 20.04"
echo "========================================="

# 更新包列表
echo "[1/6] 更新包列表..."
sudo apt-get update

# 安装基础构建工具
echo "[2/6] 安装构建工具..."
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config

# 安装 OctoMap
echo "[3/6] 安装 OctoMap..."
sudo apt-get install -y \
    liboctomap-dev \
    octovis

# 安装 Protobuf
echo "[4/6] 安装 Protobuf..."
sudo apt-get install -y \
    protobuf-compiler \
    libprotobuf-dev

# 安装 yaml-cpp
echo "[5/6] 安装 yaml-cpp..."
sudo apt-get install -y libyaml-cpp-dev

# 安装 Paho MQTT C++
echo "[6/6] 安装 Paho MQTT..."
sudo apt-get install -y \
    libssl-dev

# Paho MQTT C
cd /tmp
if [ ! -d "paho.mqtt.c" ]; then
    git clone https://github.com/eclipse/paho.mqtt.c.git
fi
cd paho.mqtt.c
git checkout v1.3.12
cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON \
    -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON
sudo cmake --build build/ --target install
sudo ldconfig

# Paho MQTT C++
cd /tmp
if [ ! -d "paho.mqtt.cpp" ]; then
    git clone https://github.com/eclipse/paho.mqtt.cpp.git
fi
cd paho.mqtt.cpp
git checkout v1.2.0
cmake -Bbuild -H. -DPAHO_BUILD_STATIC=ON \
    -DPAHO_BUILD_DOCUMENTATION=OFF -DPAHO_BUILD_SAMPLES=OFF
sudo cmake --build build/ --target install
sudo ldconfig

echo ""
echo "========================================="
echo "  依赖安装完成!"
echo "========================================="
echo ""
echo "下一步: 运行 ./scripts/build.sh 构建项目"
