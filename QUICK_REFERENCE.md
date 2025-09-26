# 开发者快速参考指南

## 快速开始

### 1. 环境配置
```bash
# 安装ARM工具链
sudo apt-get install gcc-arm-none-eabi

# 安装OpenOCD
sudo apt-get install openocd

# 安装构建工具
sudo apt-get install make cmake
```

### 2. 克隆和构建
```bash
# 克隆项目 (假设使用git)
git clone <repository-url>
cd ink_supply_system_gd32f427

# 快速构建
make DEBUG=1

# 或使用自动脚本
chmod +x build.sh
./build.sh build-make-debug
```

### 3. 烧录和调试
```bash
# 烧录程序
make flash

# 启动调试
make debug
```

## 常用命令

### Make命令
```bash
make DEBUG=1         # 构建调试版本
make                 # 构建发布版本
make clean           # 清理构建
make flash           # 烧录程序
make debug           # 启动GDB调试
make size            # 查看程序大小
make stack-usage     # 分析栈使用
make check           # 语法检查
make disasm          # 生成反汇编
```

### CMake命令
```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..  # 配置调试版本
cmake -DCMAKE_BUILD_TYPE=Release .. # 配置发布版本
make                 # 编译
make flash           # 烧录
make debug           # 调试
make size            # 查看大小
```

### 自动脚本命令
```bash
./build.sh build-make-debug     # Make调试构建
./build.sh build-make-release   # Make发布构建
./build.sh build-cmake-debug    # CMake调试构建
./build.sh build-cmake-release  # CMake发布构建
./build.sh flash                # 烧录程序
./build.sh debug                # 启动调试
./build.sh check                # 语法检查
./build.sh size                 # 查看大小
./build.sh stack                # 栈分析
./build.sh disasm               # 反汇编
./build.sh clean                # 清理
./build.sh help                 # 帮助信息
```

## 目录结构速览

```
ink_supply_system_gd32f427/
├── config/                 # 配置文件
│   ├── FreeRTOSConfig.h   # FreeRTOS配置
│   ├── lwipopts.h         # lwIP配置
│   ├── ethercat_config.h  # EtherCAT配置
│   ├── error_config.h     # 错误处理配置
│   └── gd32f427vgt6_flash.ld # 链接脚本
├── include/               # 头文件目录
├── src/                   # 源代码目录
│   ├── main.c            # 主程序
│   ├── application/      # 应用层
│   ├── middleware/       # 中间件层
│   ├── drivers/          # 驱动层
│   ├── system/           # 系统层
│   └── board/            # 板级支持
├── Makefile              # Make构建脚本
├── CMakeLists.txt        # CMake构建脚本
├── build.sh              # 自动构建脚本
└── README.md             # 项目说明
```

## 任务优先级和周期

| 任务名称 | 优先级 | 周期 | 栈大小 | 功能 |
|---------|--------|------|--------|------|
| Safety Task | 7 | 5ms | 512字节 | 安全监控 |
| Control Task | 6 | 10ms | 1024字节 | PID控制 |
| Sensor Task | 5 | 20ms | 512字节 | 传感器采集 |
| Actuator Task | 4 | 50ms | 512字节 | 执行器控制 |
| Communication Task | 3 | 50ms | 2048字节 | 网络通信 |
| HMI Task | 2 | 200ms | 1024字节 | 人机界面 |
| Config Task | 1 | 1000ms | 1024字节 | 配置管理 |

## 关键API参考

### 传感器API
```c
// 液位传感器
float get_liquid_level_sensor(uint8_t sensor_id);
bool get_sensor_fault_status(sensor_type_t type, uint8_t id);

// 压力传感器
float get_pressure_sensor(uint8_t sensor_id);

// 温度传感器
float get_pt100_temperature(uint8_t sensor_id);
```

### 执行器API
```c
// 加热器控制
sensor_result_t set_heater_power(uint8_t heater_id, float power_percent);
float get_heater_power(uint8_t heater_id);

// 泵控制
actuator_result_t set_pump_speed(uint8_t pump_id, uint16_t speed_rpm);
uint16_t get_pump_speed(uint8_t pump_id);

// 阀门控制
actuator_result_t set_valve_state(uint8_t valve_id, bool open);
bool get_valve_state(uint8_t valve_id);

// LED控制
void set_led_state(led_id_t led, bool on, led_blink_mode_t mode);
```

### 控制API
```c
// PID控制器
control_result_t pid_init(pid_controller_t* pid, float kp, float ki, float kd);
float pid_update(pid_controller_t* pid, float setpoint, float feedback);
void pid_reset(pid_controller_t* pid);

// 数字滤波
filter_result_t lowpass_filter_process(lowpass_filter_t* filter, float input, float* output);
filter_result_t moving_average_process(moving_average_filter_t* filter, float input, float* output);
```

### 安全API
```c
// 安全状态
safety_result_t safety_get_system_status(safety_status_t *status);
bool safety_is_system_safe(void);
bool safety_is_emergency_stop_active(void);

// 故障处理
safety_result_t safety_handle_fault(uint16_t fault_code, safety_level_t level);
safety_result_t safety_system_recovery(void);
```

### 配置API
```c
// 配置读写
config_result_t config_get_system(system_config_t *config);
config_result_t config_set_system(const system_config_t *config);
config_result_t config_save_to_flash(void);
config_result_t config_load_from_flash(void);
```

## 调试技巧

### 1. 串口调试
```c
// 在代码中添加调试输出
printf("Debug: sensor value = %.2f\n", sensor_value);
```

### 2. LED状态指示
```c
// 使用LED指示系统状态
set_led_state(LED_ERROR, true, LED_BLINK_FAST);  // 错误指示
set_led_state(LED_RUN, true, LED_STEADY);        // 运行指示
```

### 3. GDB调试
```bash
# 启动GDB
make debug

# 常用GDB命令
(gdb) break main           # 在main函数设置断点
(gdb) continue             # 继续执行
(gdb) step                 # 单步执行
(gdb) print variable_name  # 打印变量值
(gdb) info registers       # 查看寄存器
(gdb) backtrace           # 查看调用栈
```

### 4. 性能分析
```bash
# 查看程序大小
make size

# 分析栈使用
make stack-usage

# 生成反汇编
make disasm
```

## 常见问题解决

### 编译错误
1. **工具链问题**: 确认arm-none-eabi-gcc已安装
2. **路径问题**: 检查头文件包含路径
3. **库文件问题**: 确认第三方库完整

### 烧录问题
1. **硬件连接**: 检查ST-Link连接
2. **目标板供电**: 确认目标板正常供电
3. **OpenOCD配置**: 检查openocd配置文件

### 运行问题
1. **看门狗超时**: 检查任务执行时间
2. **栈溢出**: 增大任务栈大小
3. **内存不足**: 检查动态内存分配

### 通信问题
1. **网络配置**: 检查IP地址配置
2. **协议栈初始化**: 确认lwIP初始化成功
3. **EtherCAT配置**: 检查ESC配置参数

## 性能优化建议

### 1. 编译优化
- 发布版本使用-Os优化
- 使用LTO链接时优化
- 启用函数和数据段分离

### 2. 内存优化
- 合理设置任务栈大小
- 减少全局变量使用
- 使用静态内存分配

### 3. 实时性优化
- 合理设置任务优先级
- 减少中断处理时间
- 使用DMA传输数据

### 4. 功耗优化
- 使用低功耗模式
- 关闭不用的外设时钟
- 优化任务调度周期

## 扩展开发指南

### 添加新传感器
1. 在`src/drivers/sensors/`创建驱动文件
2. 在`include/drivers/sensors/`添加头文件
3. 在传感器任务中集成新传感器
4. 更新配置和文档

### 添加新执行器
1. 在`src/drivers/actuators/`创建驱动文件
2. 在`include/drivers/actuators/`添加头文件
3. 在执行器任务中集成新执行器
4. 更新安全检查和配置

### 添加新通信协议
1. 在`src/application/communication/`添加协议实现
2. 在通信任务中集成新协议
3. 更新网络配置和文档

### 添加新控制算法
1. 在`src/middleware/control/`实现算法
2. 在控制任务中集成算法
3. 添加参数配置和调试接口

---

**提示**: 开发过程中遇到问题，请先查阅README.md和相关头文件注释，如需进一步帮助，请查看项目文档或联系开发团队。