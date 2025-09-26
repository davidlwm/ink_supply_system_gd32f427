# 供墨系统控制板卡 GD32F427 项目构建脚本
# Makefile for Ink Supply System Control Board (GD32F427)
# Version: V4.0
# Date: 2025-09-27

# 项目名称
PROJECT_NAME = ink_supply_system_gd32f427

# 目标芯片
MCU = GD32F427VGT6
ARCH = cortex-m4

# 编译器配置
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
AR = $(PREFIX)ar
GDB = $(PREFIX)gdb

# 编译标志
CFLAGS = -mcpu=$(ARCH) -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS += -fdata-sections -ffunction-sections -Wall -fstack-usage
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# 调试/发布配置
ifdef DEBUG
	CFLAGS += -g3 -DDEBUG -O0
	BUILD_DIR = build/debug
else
	CFLAGS += -Os -DNDEBUG
	BUILD_DIR = build/release
endif

# 定义
DEFS = -DUSE_STDPERIPH_DRIVER -DGD32F427 -DUSE_FREERTOS

# 包含路径 (8周v4标准结构)
INCLUDES = -Iinclude
INCLUDES += -Iinclude/config
INCLUDES += -Iinclude/system
INCLUDES += -Iinclude/app
INCLUDES += -Iinclude/middleware
INCLUDES += -Iinclude/hal
INCLUDES += -Iinclude/drivers
INCLUDES += -Iinclude/drivers/sensors
INCLUDES += -Iinclude/drivers/actuators

# 第三方库包含路径
INCLUDES += -Ilib/GD32F4xx_standard_peripheral/Include
INCLUDES += -Ilib/CMSIS/Include
INCLUDES += -Ilib/CMSIS/Device/GD32F4xx/Include
INCLUDES += -Ilib/FreeRTOS/Source/include
INCLUDES += -Ilib/FreeRTOS/Source/portable/GCC/ARM_CM4F
INCLUDES += -Ilib/lwip/src/include
INCLUDES += -Ilib/lwip/src/include/ipv4
INCLUDES += -Ilib/lwip/system
INCLUDES += -Ilib/EtherCAT_Stack/inc

# 源文件路径 (8周v4标准结构)
SOURCES_DIR = src

# 应用层源文件
APP_SOURCES = $(wildcard $(SOURCES_DIR)/app/*/*.c)

# 中间件层源文件 (filter, pid, tasks)
MIDDLEWARE_SOURCES = $(wildcard $(SOURCES_DIR)/middleware/*.c)

# HAL层源文件
HAL_SOURCES = $(wildcard $(SOURCES_DIR)/hal/*.c)

# 驱动层源文件 (递归搜索子目录)
DRIVER_SOURCES = $(wildcard $(SOURCES_DIR)/drivers/*/*.c)

# 主程序源文件
MAIN_SOURCES = $(SOURCES_DIR)/main.c

# 第三方库源文件
# GD32F4xx标准外设库
LIB_SOURCES = $(wildcard lib/GD32F4xx_standard_peripheral/Source/*.c)

# FreeRTOS源文件
LIB_SOURCES += lib/FreeRTOS/Source/tasks.c
LIB_SOURCES += lib/FreeRTOS/Source/list.c
LIB_SOURCES += lib/FreeRTOS/Source/queue.c
LIB_SOURCES += lib/FreeRTOS/Source/timers.c
LIB_SOURCES += lib/FreeRTOS/Source/event_groups.c
LIB_SOURCES += lib/FreeRTOS/Source/stream_buffer.c
LIB_SOURCES += lib/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
LIB_SOURCES += lib/FreeRTOS/Source/portable/MemMang/heap_4.c

# lwIP源文件
LIB_SOURCES += $(wildcard lib/lwip/src/core/*.c)
LIB_SOURCES += $(wildcard lib/lwip/src/core/ipv4/*.c)
LIB_SOURCES += $(wildcard lib/lwip/src/api/*.c)
LIB_SOURCES += $(wildcard lib/lwip/src/netif/*.c)
LIB_SOURCES += $(wildcard lib/lwip/system/*.c)

# EtherCAT源文件
LIB_SOURCES += $(wildcard lib/EtherCAT_Stack/src/*.c)

# 汇编源文件
ASM_SOURCES = lib/CMSIS/Device/GD32F4xx/Source/GCC/startup_gd32f427.s

# 所有源文件 (8周v4标准)
C_SOURCES = $(MAIN_SOURCES) $(APP_SOURCES) $(MIDDLEWARE_SOURCES) $(HAL_SOURCES) $(DRIVER_SOURCES) $(LIB_SOURCES)

# 链接器脚本
LDSCRIPT = config/gd32f427vgt6_flash.ld

# 链接器标志
LDFLAGS = -mcpu=$(ARCH) -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
LDFLAGS += -specs=nano.specs -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(PROJECT_NAME).map,--cref -Wl,--gc-sections
LDFLAGS += -static -Wl,--start-group -lc -lm -Wl,--end-group

# 默认目标
all: $(BUILD_DIR)/$(PROJECT_NAME).elf $(BUILD_DIR)/$(PROJECT_NAME).hex $(BUILD_DIR)/$(PROJECT_NAME).bin

# 创建对象文件列表
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# 编译C源文件
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "CC $<"
	@$(CC) -c $(CFLAGS) $(DEFS) $(INCLUDES) $< -o $@

# 编译汇编源文件
$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "AS $<"
	@$(AS) -c $(CFLAGS) $(DEFS) $< -o $@

# 链接生成ELF文件
$(BUILD_DIR)/$(PROJECT_NAME).elf: $(OBJECTS) Makefile
	@echo "LD $@"
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	@$(SZ) $@

# 生成HEX文件
$(BUILD_DIR)/$(PROJECT_NAME).hex: $(BUILD_DIR)/$(PROJECT_NAME).elf | $(BUILD_DIR)
	@echo "OBJCOPY $@"
	@$(CP) -O ihex $< $@

# 生成BIN文件
$(BUILD_DIR)/$(PROJECT_NAME).bin: $(BUILD_DIR)/$(PROJECT_NAME).elf | $(BUILD_DIR)
	@echo "OBJCOPY $@"
	@$(CP) -O binary -S $< $@

# 创建构建目录
$(BUILD_DIR):
	mkdir -p $@

# 清理
clean:
	@echo "Cleaning..."
	@rm -fR build

# 烧录程序 (需要安装OpenOCD)
flash: $(BUILD_DIR)/$(PROJECT_NAME).elf
	@echo "Flashing $(PROJECT_NAME).elf"
	openocd -f interface/stlink.cfg -f target/gd32f4x.cfg -c "program $(BUILD_DIR)/$(PROJECT_NAME).elf verify reset exit"

# 调试
debug: $(BUILD_DIR)/$(PROJECT_NAME).elf
	@echo "Starting GDB debug session"
	$(GDB) $(BUILD_DIR)/$(PROJECT_NAME).elf

# 查看大小
size: $(BUILD_DIR)/$(PROJECT_NAME).elf
	@$(SZ) $(BUILD_DIR)/$(PROJECT_NAME).elf

# 分析栈使用
stack-usage: all
	@echo "Stack usage analysis:"
	@find $(BUILD_DIR) -name "*.su" -exec cat {} \; | sort -k2 -nr | head -20

# 检查语法
check:
	@echo "Checking syntax..."
	@$(CC) $(CFLAGS) $(DEFS) $(INCLUDES) -fsyntax-only $(C_SOURCES)

# 生成反汇编
disasm: $(BUILD_DIR)/$(PROJECT_NAME).elf
	@echo "Generating disassembly..."
	@$(PREFIX)objdump -d $< > $(BUILD_DIR)/$(PROJECT_NAME).dis

# 显示帮助
help:
	@echo "Available targets:"
	@echo "  all         - Build the project"
	@echo "  clean       - Clean build files"
	@echo "  flash       - Flash the program to target"
	@echo "  debug       - Start GDB debug session"
	@echo "  size        - Show program size"
	@echo "  stack-usage - Analyze stack usage"
	@echo "  check       - Check syntax only"
	@echo "  disasm      - Generate disassembly"
	@echo "  help        - Show this help"
	@echo ""
	@echo "Build configurations:"
	@echo "  make DEBUG=1   - Debug build"
	@echo "  make           - Release build"

# 依赖文件
-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all clean flash debug size stack-usage check disasm help