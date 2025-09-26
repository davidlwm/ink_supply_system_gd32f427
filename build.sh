#!/bin/bash
# 构建脚本 - GD32F427供墨系统控制板卡
# Build script for Ink Supply System Control Board (GD32F427)
# Version: V4.0
# Date: 2025-09-27

# 项目配置
PROJECT_NAME="ink_supply_system_gd32f427"
PROJECT_VERSION="4.0.0"
BUILD_DIR="build"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_message() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

# 检查工具链
check_toolchain() {
    print_message $BLUE "检查ARM工具链..."

    if ! command -v arm-none-eabi-gcc &> /dev/null; then
        print_message $RED "错误: arm-none-eabi-gcc 未找到"
        print_message $YELLOW "请安装ARM GNU工具链"
        exit 1
    fi

    if ! command -v arm-none-eabi-gdb &> /dev/null; then
        print_message $RED "错误: arm-none-eabi-gdb 未找到"
        print_message $YELLOW "请安装ARM GNU工具链"
        exit 1
    fi

    local gcc_version=$(arm-none-eabi-gcc --version | head -n1)
    print_message $GREEN "工具链检查通过: $gcc_version"
}

# 创建构建目录
create_build_dir() {
    if [ ! -d "$BUILD_DIR" ]; then
        mkdir -p "$BUILD_DIR"
        print_message $GREEN "创建构建目录: $BUILD_DIR"
    fi
}

# 清理构建
clean_build() {
    print_message $BLUE "清理构建..."
    rm -rf "$BUILD_DIR"
    print_message $GREEN "构建清理完成"
}

# Make构建
build_make() {
    local build_type=$1

    print_message $BLUE "使用Make构建 ($build_type)..."

    create_build_dir

    if [ "$build_type" = "debug" ]; then
        make DEBUG=1 -j$(nproc)
    else
        make -j$(nproc)
    fi

    if [ $? -eq 0 ]; then
        print_message $GREEN "Make构建成功"
    else
        print_message $RED "Make构建失败"
        exit 1
    fi
}

# CMake构建
build_cmake() {
    local build_type=$1

    print_message $BLUE "使用CMake构建 ($build_type)..."

    create_build_dir
    cd "$BUILD_DIR"

    if [ "$build_type" = "debug" ]; then
        cmake -DCMAKE_BUILD_TYPE=Debug ..
    else
        cmake -DCMAKE_BUILD_TYPE=Release ..
    fi

    if [ $? -eq 0 ]; then
        make -j$(nproc)
        if [ $? -eq 0 ]; then
            print_message $GREEN "CMake构建成功"
        else
            print_message $RED "CMake构建失败"
            exit 1
        fi
    else
        print_message $RED "CMake配置失败"
        exit 1
    fi

    cd ..
}

# 烧录程序
flash_program() {
    print_message $BLUE "烧录程序到目标板..."

    if [ ! -f "$BUILD_DIR/${PROJECT_NAME}.elf" ] && [ ! -f "${PROJECT_NAME}.elf" ]; then
        print_message $RED "错误: 找不到ELF文件，请先构建项目"
        exit 1
    fi

    if command -v openocd &> /dev/null; then
        if [ -f "$BUILD_DIR/${PROJECT_NAME}.elf" ]; then
            openocd -f interface/stlink.cfg -f target/gd32f4x.cfg \
                    -c "program $BUILD_DIR/${PROJECT_NAME}.elf verify reset exit"
        else
            openocd -f interface/stlink.cfg -f target/gd32f4x.cfg \
                    -c "program ${PROJECT_NAME}.elf verify reset exit"
        fi

        if [ $? -eq 0 ]; then
            print_message $GREEN "程序烧录成功"
        else
            print_message $RED "程序烧录失败"
            exit 1
        fi
    else
        print_message $RED "错误: openocd 未找到"
        print_message $YELLOW "请安装OpenOCD调试工具"
        exit 1
    fi
}

# 启动调试
start_debug() {
    print_message $BLUE "启动GDB调试会话..."

    if [ ! -f "$BUILD_DIR/${PROJECT_NAME}.elf" ] && [ ! -f "${PROJECT_NAME}.elf" ]; then
        print_message $RED "错误: 找不到ELF文件，请先构建项目"
        exit 1
    fi

    if [ -f "$BUILD_DIR/${PROJECT_NAME}.elf" ]; then
        arm-none-eabi-gdb "$BUILD_DIR/${PROJECT_NAME}.elf"
    else
        arm-none-eabi-gdb "${PROJECT_NAME}.elf"
    fi
}

# 检查语法
check_syntax() {
    print_message $BLUE "检查代码语法..."

    if command -v make &> /dev/null; then
        make check
        if [ $? -eq 0 ]; then
            print_message $GREEN "语法检查通过"
        else
            print_message $RED "语法检查失败"
            exit 1
        fi
    else
        print_message $RED "错误: make 未找到"
        exit 1
    fi
}

# 显示程序大小
show_size() {
    print_message $BLUE "显示程序大小信息..."

    if [ -f "$BUILD_DIR/${PROJECT_NAME}.elf" ]; then
        arm-none-eabi-size "$BUILD_DIR/${PROJECT_NAME}.elf"
    elif [ -f "${PROJECT_NAME}.elf" ]; then
        arm-none-eabi-size "${PROJECT_NAME}.elf"
    else
        print_message $RED "错误: 找不到ELF文件，请先构建项目"
        exit 1
    fi
}

# 栈使用分析
analyze_stack() {
    print_message $BLUE "分析栈使用情况..."

    if [ -d "$BUILD_DIR" ]; then
        find "$BUILD_DIR" -name "*.su" -exec cat {} \; | sort -k2 -nr | head -20
    else
        print_message $RED "错误: 找不到构建目录，请先构建项目"
        exit 1
    fi
}

# 生成反汇编
generate_disasm() {
    print_message $BLUE "生成反汇编文件..."

    if [ -f "$BUILD_DIR/${PROJECT_NAME}.elf" ]; then
        arm-none-eabi-objdump -d "$BUILD_DIR/${PROJECT_NAME}.elf" > "$BUILD_DIR/${PROJECT_NAME}.dis"
        print_message $GREEN "反汇编文件已生成: $BUILD_DIR/${PROJECT_NAME}.dis"
    elif [ -f "${PROJECT_NAME}.elf" ]; then
        arm-none-eabi-objdump -d "${PROJECT_NAME}.elf" > "${PROJECT_NAME}.dis"
        print_message $GREEN "反汇编文件已生成: ${PROJECT_NAME}.dis"
    else
        print_message $RED "错误: 找不到ELF文件，请先构建项目"
        exit 1
    fi
}

# 显示帮助
show_help() {
    echo "GD32F427供墨系统控制板卡构建脚本"
    echo "版本: $PROJECT_VERSION"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  build-make-debug     使用Make构建调试版本"
    echo "  build-make-release   使用Make构建发布版本"
    echo "  build-cmake-debug    使用CMake构建调试版本"
    echo "  build-cmake-release  使用CMake构建发布版本"
    echo "  flash               烧录程序到目标板"
    echo "  debug               启动GDB调试会话"
    echo "  check               检查代码语法"
    echo "  size                显示程序大小"
    echo "  stack               分析栈使用情况"
    echo "  disasm              生成反汇编文件"
    echo "  clean               清理构建文件"
    echo "  help                显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 build-make-debug   # 使用Make构建调试版本"
    echo "  $0 build-cmake-release # 使用CMake构建发布版本"
    echo "  $0 flash              # 烧录程序"
    echo "  $0 debug              # 启动调试"
}

# 主函数
main() {
    print_message $GREEN "=== GD32F427供墨系统控制板卡构建脚本 ==="
    print_message $GREEN "=== 版本: $PROJECT_VERSION ==="
    echo ""

    # 检查工具链
    check_toolchain

    # 处理命令行参数
    case "$1" in
        "build-make-debug")
            build_make "debug"
            ;;
        "build-make-release")
            build_make "release"
            ;;
        "build-cmake-debug")
            build_cmake "debug"
            ;;
        "build-cmake-release")
            build_cmake "release"
            ;;
        "flash")
            flash_program
            ;;
        "debug")
            start_debug
            ;;
        "check")
            check_syntax
            ;;
        "size")
            show_size
            ;;
        "stack")
            analyze_stack
            ;;
        "disasm")
            generate_disasm
            ;;
        "clean")
            clean_build
            ;;
        "help")
            show_help
            ;;
        *)
            print_message $YELLOW "未知选项: $1"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

# 运行主函数
main "$@"