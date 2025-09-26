# CMake构建文件 - GD32F427供墨系统控制板卡
# CMakeLists.txt for Ink Supply System Control Board (GD32F427)
# Version: V4.0
# Date: 2025-09-27

cmake_minimum_required(VERSION 3.16)

# 项目信息
project(ink_supply_system_gd32f427
        VERSION 4.0.0
        LANGUAGES C ASM
        DESCRIPTION "Ink Supply System Control Board for GD32F427")

# 设置C标准
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 工具链设置
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 交叉编译工具链
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)

# 防止CMake添加-rdynamic链接选项
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

# MCU设置
set(MCU cortex-m4)
set(FLOAT_ABI hard)
set(FPU fpv4-sp-d16)

# 编译器标志
set(MCU_FLAGS "-mcpu=${MCU} -mthumb -mfpu=${FPU} -mfloat-abi=${FLOAT_ABI}")
set(COMMON_FLAGS "${MCU_FLAGS} -fdata-sections -ffunction-sections -Wall -fstack-usage")

# 调试/发布配置
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_C_FLAGS "${COMMON_FLAGS} -g3 -DDEBUG -O0")
    message(STATUS "Build type: Debug")
else()
    set(CMAKE_C_FLAGS "${COMMON_FLAGS} -Os -DNDEBUG")
    message(STATUS "Build type: Release")
endif()

# 链接器标志
set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} -specs=nano.specs -Wl,-Map=${PROJECT_NAME}.map,--cref -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group")

# 定义
add_definitions(-DUSE_STDPERIPH_DRIVER -DGD32F427 -DUSE_FREERTOS)

# 链接器脚本
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/config/gd32f427vgt6_flash.ld)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${LINKER_SCRIPT}")

# 包含目录
include_directories(
    include
    include/application/sensor
    include/application/actuator
    include/application/control
    include/application/communication
    include/application/hmi
    include/application/safety
    include/application/config
    include/middleware/rtos
    include/middleware/filter
    include/middleware/control
    include/drivers/hal
    include/drivers/bsp
    include/system
    config

    # 第三方库
    lib/GD32F4xx_standard_peripheral/Include
    lib/CMSIS/Include
    lib/CMSIS/Device/GD32F4xx/Include
    lib/FreeRTOS/Source/include
    lib/FreeRTOS/Source/portable/GCC/ARM_CM4F
    lib/lwip/src/include
    lib/lwip/src/include/ipv4
    lib/lwip/system
    lib/EtherCAT_Stack/inc
)

# 源文件收集
file(GLOB_RECURSE APP_SOURCES
    "src/application/*.c"
)

file(GLOB_RECURSE MIDDLEWARE_SOURCES
    "src/middleware/*.c"
)

file(GLOB_RECURSE DRIVER_SOURCES
    "src/drivers/*.c"
)

file(GLOB_RECURSE SYSTEM_SOURCES
    "src/system/*.c"
    "src/board/*.c"
)

# 第三方库源文件
file(GLOB LIB_GD32_SOURCES
    "lib/GD32F4xx_standard_peripheral/Source/*.c"
)

set(LIB_FREERTOS_SOURCES
    lib/FreeRTOS/Source/tasks.c
    lib/FreeRTOS/Source/list.c
    lib/FreeRTOS/Source/queue.c
    lib/FreeRTOS/Source/timers.c
    lib/FreeRTOS/Source/event_groups.c
    lib/FreeRTOS/Source/stream_buffer.c
    lib/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    lib/FreeRTOS/Source/portable/MemMang/heap_4.c
)

file(GLOB_RECURSE LIB_LWIP_SOURCES
    "lib/lwip/src/core/*.c"
    "lib/lwip/src/core/ipv4/*.c"
    "lib/lwip/src/api/*.c"
    "lib/lwip/src/netif/*.c"
    "lib/lwip/system/*.c"
)

file(GLOB_RECURSE LIB_ETHERCAT_SOURCES
    "lib/EtherCAT_Stack/src/*.c"
)

# 汇编源文件
set(ASM_SOURCES
    lib/CMSIS/Device/GD32F4xx/Source/GCC/startup_gd32f427.s
)

# 所有源文件
set(ALL_SOURCES
    src/main.c
    ${APP_SOURCES}
    ${MIDDLEWARE_SOURCES}
    ${DRIVER_SOURCES}
    ${SYSTEM_SOURCES}
    ${LIB_GD32_SOURCES}
    ${LIB_FREERTOS_SOURCES}
    ${LIB_LWIP_SOURCES}
    ${LIB_ETHERCAT_SOURCES}
    ${ASM_SOURCES}
)

# 创建可执行文件
add_executable(${PROJECT_NAME}.elf ${ALL_SOURCES})

# 设置汇编文件编译标志
set_property(SOURCE ${ASM_SOURCES} PROPERTY LANGUAGE ASM)

# 生成二进制文件
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Building ${PROJECT_NAME}.hex and ${PROJECT_NAME}.bin"
)

# 打印尺寸信息
add_custom_target(size ALL
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Program size:"
)

# 烧录目标
add_custom_target(flash
    COMMAND openocd -f interface/stlink.cfg -f target/gd32f4x.cfg -c "program ${PROJECT_NAME}.elf verify reset exit"
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Flashing ${PROJECT_NAME}.elf"
)

# 调试目标
add_custom_target(debug
    COMMAND arm-none-eabi-gdb ${PROJECT_NAME}.elf
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Starting GDB debug session"
)

# 反汇编目标
add_custom_target(disasm
    COMMAND ${CMAKE_OBJDUMP} -d ${PROJECT_NAME}.elf > ${PROJECT_NAME}.dis
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Generating disassembly"
)

# 清理目标
add_custom_target(clean-all
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}
    COMMENT "Cleaning all build files"
)

# 栈使用分析目标
add_custom_target(stack-usage
    COMMAND find ${CMAKE_BINARY_DIR} -name "*.su" -exec cat {} + | sort -k2 -nr | head -20
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Stack usage analysis"
)

# 语法检查目标
add_custom_target(check
    COMMAND ${CMAKE_C_COMPILER} ${CMAKE_C_FLAGS} -fsyntax-only ${ALL_SOURCES}
    COMMENT "Checking syntax"
)

# 项目信息打印
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "MCU: GD32F427VGT6")
message(STATUS "Architecture: ${MCU}")
message(STATUS "Float ABI: ${FLOAT_ABI}")
message(STATUS "FPU: ${FPU}")
message(STATUS "C Standard: C${CMAKE_C_STANDARD}")

# 输出变量信息
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Optimization: Debug (-O0)")
else()
    message(STATUS "Optimization: Release (-Os)")
endif()

# 输出源文件统计
list(LENGTH APP_SOURCES APP_COUNT)
list(LENGTH MIDDLEWARE_SOURCES MIDDLEWARE_COUNT)
list(LENGTH DRIVER_SOURCES DRIVER_COUNT)
list(LENGTH SYSTEM_SOURCES SYSTEM_COUNT)

message(STATUS "Application sources: ${APP_COUNT}")
message(STATUS "Middleware sources: ${MIDDLEWARE_COUNT}")
message(STATUS "Driver sources: ${DRIVER_COUNT}")
message(STATUS "System sources: ${SYSTEM_COUNT}")

# 帮助信息
add_custom_target(help
    COMMAND ${CMAKE_COMMAND} -E echo "Available targets:"
    COMMAND ${CMAKE_COMMAND} -E echo "  all         - Build the project"
    COMMAND ${CMAKE_COMMAND} -E echo "  flash       - Flash the program to target"
    COMMAND ${CMAKE_COMMAND} -E echo "  debug       - Start GDB debug session"
    COMMAND ${CMAKE_COMMAND} -E echo "  size        - Show program size"
    COMMAND ${CMAKE_COMMAND} -E echo "  stack-usage - Analyze stack usage"
    COMMAND ${CMAKE_COMMAND} -E echo "  check       - Check syntax only"
    COMMAND ${CMAKE_COMMAND} -E echo "  disasm      - Generate disassembly"
    COMMAND ${CMAKE_COMMAND} -E echo "  clean-all   - Clean all build files"
    COMMAND ${CMAKE_COMMAND} -E echo "  help        - Show this help"
    COMMAND ${CMAKE_COMMAND} -E echo ""
    COMMAND ${CMAKE_COMMAND} -E echo "Build configurations:"
    COMMAND ${CMAKE_COMMAND} -E echo "  cmake -DCMAKE_BUILD_TYPE=Debug .."
    COMMAND ${CMAKE_COMMAND} -E echo "  cmake -DCMAKE_BUILD_TYPE=Release .."
    COMMENT "Help information"
)

# 设置默认目标
if(NOT TARGET)
    set(TARGET all)
endif()