#!/bin/bash

# 3D探索系统构建脚本

set -e

BUILD_TYPE=${1:-Release}
BUILD_DIR="build"

echo "========================================="
echo "  3D 自主探索系统 - 构建脚本"
echo "  构建类型: $BUILD_TYPE"
echo "========================================="

# 创建构建目录
mkdir -p $BUILD_DIR
cd $BUILD_DIR

# 运行CMake
echo "[1/3] 运行 CMake..."
cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE

# 编译
echo "[2/3] 编译..."
make -j$(nproc)

# 完成
echo "[3/3] 构建完成!"
echo ""
echo "可执行文件: $BUILD_DIR/exploration_3d"
echo ""
echo "运行方式:"
echo "  ./exploration_3d                           # 使用默认配置"
echo "  ./exploration_3d -c config/custom.yaml     # 指定配置文件"
echo "  ./exploration_3d -m direct                 # 使用直线模式"
echo "  ./exploration_3d -g                        # 生成默认配置"
echo "  ./exploration_3d -h                        # 显示帮助"
